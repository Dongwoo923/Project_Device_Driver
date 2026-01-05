// ds1302_oled.c  (I2C SSD1306 128x64, DS1302 bitbang, no RST pin)
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/types.h>
extern void rotary_irq_enable(bool on);
// UI 주기 / 센서 주기 / blink 주기
#define UI_TICK_MS   50
#define SENSE_TICK_MS 1000

#define DRIVER_NAME   "ds1302_oled"
#define CLASS_NAME    "ds1302_oled_class"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kkk");
MODULE_DESCRIPTION("DS1302(bitbang) -> SSD1306(I2C) show date/time every 1s");

struct ds_time;  // 전방 선언
static int ds1302_read_time(struct ds_time *t);
static int ds1302_set_datetime(const struct ds_time *t);
static unsigned long last_sense_j = 0;
static unsigned long last_blink_j = 0;
static struct ds_time t_cache;

static int temp_cache = -1, humi_cache = -1;
static bool cache_ok = false;
static unsigned long last_sense_j;

//oled/ds1302 모듈에서 dht11값을 직접 호출
extern int dht11_read_values(int *temp, int *humi);
extern int rotary_get_event(void);
enum rotary_evt_type {
    ROT_EV_NONE = 0,
    ROT_EV_CW,
    ROT_EV_CCW,
    ROT_EV_BTN_DOWN,
    ROT_EV_BTN_UP,
};
static dev_t dev_num;
static struct cdev ds_cdev;
static struct class *ds_class;
static struct device *ds_dev;

static DEFINE_MUTEX(ds_lock); // DS1302 동시접근 방지

// -------------------- DS1302 GPIO params --------------------
static int ds_ce_gpio  = 12;  // DS1302 CE(RST)
static int ds_clk_gpio = 5;   // DS1302 CLK
static int ds_dat_gpio = 6;   // DS1302 DAT (I/O)
module_param(ds_ce_gpio, int, 0644);
module_param(ds_clk_gpio, int, 0644);
module_param(ds_dat_gpio, int, 0644);

// -------------------- I2C OLED params (i2c-gpio bus=8, addr=0x3C) --------------------
static int i2c_bus  = 1;
static int i2c_addr = 0x3C;
module_param(i2c_bus, int, 0644);
module_param(i2c_addr, int, 0644);

// (선택) 모듈 로딩 시 DS1302 초기 세팅하고 싶으면 init_datetime=YYYYMMDDhhmmss
static char *init_datetime = NULL;
module_param(init_datetime, charp, 0644);

// I2C 객체
static struct i2c_adapter *oled_adap;
static struct i2c_client  *oled_i2c;

// -------------------- OLED framebuffer --------------------
#define OLED_W   128
#define OLED_H   64
#define OLED_BUF (OLED_W * OLED_H / 8)

static u8 fb[OLED_BUF];
static struct delayed_work tick_work;

// -------------------- SET mode state --------------------
enum ui_mode {
    UI_NORMAL = 0,
    UI_SET,
};

enum set_field {
    FLD_YEAR = 0,
    FLD_MON,
    FLD_MDAY,
    FLD_HOUR,
    FLD_MIN,
    FLD_SEC,
    FLD_MAX
};
struct ds_time {
    u8 sec, min, hour;
    u8 mday, mon, wday;
    u8 year; // 0..99 (display: 2000+year)
};

static enum ui_mode  g_mode = UI_NORMAL;
static enum set_field g_field = FLD_HOUR;

static struct ds_time g_edit;        // SET 모드에서 편집하는 시간
static bool g_blink_on = true;
static unsigned long g_last_blink_j = 0;


#define BTN_LONG_MS 1200
#define BLINK_MS    500

static inline int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int days_in_month(int year4, int mon)
{
    static const int mdays[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    int d = mdays[clamp_int(mon,1,12)-1];

    // leap year (2000~2099 범위 가정)
    if (mon == 2) {
        int leap = ((year4 % 4) == 0); // 2000~2099에서는 이 조건이면 충분
        if (leap) d = 29;
    }
    return d;
}

/* 필드 증감 */
static void edit_add(struct ds_time *t, int delta)
{
    int year4 = 2000 + t->year;
    int v;

    switch (g_field) {
    case FLD_YEAR:
        v = year4 + delta;
        v = clamp_int(v, 2000, 2099);
        t->year = (u8)(v - 2000);
        // 날짜 유효성 보정
        t->mday = clamp_int(t->mday, 1, days_in_month(v, t->mon));
        break;

    case FLD_MON:
        v = (int)t->mon + delta;
        if (v < 1) v = 12;
        if (v > 12) v = 1;
        t->mon = (u8)v;
        t->mday = clamp_int(t->mday, 1, days_in_month(2000 + t->year, t->mon));
        break;

    case FLD_MDAY:
        v = (int)t->mday + delta;
        {
            int dim = days_in_month(2000 + t->year, t->mon);
            if (v < 1) v = dim;
            if (v > dim) v = 1;
            t->mday = (u8)v;
        }
        break;

    case FLD_HOUR:
        v = (int)t->hour + delta;
        if (v < 0) v = 23;
        if (v > 23) v = 0;
        t->hour = (u8)v;
        break;

    case FLD_MIN:
        v = (int)t->min + delta;
        if (v < 0) v = 59;
        if (v > 59) v = 0;
        t->min = (u8)v;
        break;

    case FLD_SEC:
        v = (int)t->sec + delta;
        if (v < 0) v = 59;
        if (v > 59) v = 0;
        t->sec = (u8)v;
        break;

    default:
        break;
    }
}

static void field_next(void)
{
    switch (g_field) {
    case FLD_YEAR: g_field = FLD_MON;  break;
    case FLD_MON:  g_field = FLD_MDAY; break;
    case FLD_MDAY: g_field = FLD_HOUR; break;
    case FLD_HOUR: g_field = FLD_MIN;  break;
    case FLD_MIN:  g_field = FLD_SEC;  break;
    default:       g_field = FLD_YEAR; break; // FLD_SEC 포함해서 여기로
    }
}

/* NORMAL->SET 진입: 현재 RTC 읽어서 편집 버퍼에 로드 */
static int enter_set_mode(void)
{
    struct ds_time t;
    int ret;

    mutex_lock(&ds_lock);
    ret = ds1302_read_time(&t);
    mutex_unlock(&ds_lock);
    if (ret) 
        return ret;

    g_edit = t;
    g_mode = UI_SET;
    g_field = FLD_YEAR;
    rotary_irq_enable(true);

    g_blink_on = true;
    g_last_blink_j = jiffies;
    return 0;
}

static int save_and_exit_set_mode(void)
{
    int ret;

    /* 요일은 지금 구조상 1固定이니, 최소한 1로 유지 */
    if (g_edit.wday == 0) g_edit.wday = 1;

   mutex_lock(&ds_lock);
    ret = ds1302_set_datetime(&g_edit);
    mutex_unlock(&ds_lock);

    if (ret)
        return ret;   // SET 유지, 로터리 IRQ도 유지

    rotary_irq_enable(false);
    g_mode = UI_NORMAL;
    return 0;
}

/* dt: "YYYY-MM-DD" (10 chars), tm: "HH:MM:SS" (8 chars)
 * 선택 필드를 blink_off일 때 공백으로 덮어쓴다.
 */
static void apply_blink_mask(char *dt, char *tm, bool blink_on)
{
    if (blink_on) return;

    switch (g_field) {
    case FLD_YEAR:
        dt[0] = dt[1] = dt[2] = dt[3] = ' ';
        break;
    case FLD_MON:
        dt[5] = dt[6] = ' ';
        break;
    case FLD_MDAY:
        dt[8] = dt[9] = ' ';
        break;
    case FLD_HOUR:
        tm[0] = tm[1] = ' ';
        break;
    case FLD_MIN:
        tm[3] = tm[4] = ' ';
        break;
    case FLD_SEC:
        tm[6] = tm[7] = ' ';
        break;
    default:
        break;
    }
}


// -------------------- 6x8 font: digits + ':' + '-' + ' ' --------------------
static const u8 font6x8[11][6] = {
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, //0
    {0x00,0x42,0x7F,0x40,0x00,0x00}, //1
    {0x62,0x51,0x49,0x49,0x46,0x00}, //2
    {0x22,0x49,0x49,0x49,0x36,0x00}, //3
    {0x18,0x14,0x12,0x7F,0x10,0x00}, //4
    {0x2F,0x49,0x49,0x49,0x31,0x00}, //5
    {0x3E,0x49,0x49,0x49,0x32,0x00}, //6
    {0x01,0x71,0x09,0x05,0x03,0x00}, //7
    {0x36,0x49,0x49,0x49,0x36,0x00}, //8
    {0x26,0x49,0x49,0x49,0x3E,0x00}, //9
    {0x00,0x36,0x36,0x00,0x00,0x00}, //:
};

// 추가 글리프(6x8): ' ' 'T' 'H' 'C' '%'
static const u8 glyph_space[6]   = { 0x00,0x00,0x00,0x00,0x00,0x00 };
static const u8 glyph_T[6]       = { 0x01,0x01,0x7F,0x01,0x01,0x00 };
static const u8 glyph_H[6]       = { 0x7F,0x08,0x08,0x08,0x7F,0x00 };
static const u8 glyph_C[6]       = { 0x3E,0x41,0x41,0x41,0x22,0x00 };
static const u8 glyph_percent[6] = { 0x13,0x2B,0x04,0x32,0x31,0x00 };
static const u8 glyph_S[6] = { 0x26,0x49,0x49,0x49,0x32,0x00 }; // 'S'
static const u8 glyph_E[6] = { 0x7F,0x49,0x49,0x49,0x41,0x00 }; // 'E'
// 너가 이미 추가해둔 '-' 글리프
static const u8 glyph_dash[6]    = { 0x08,0x08,0x08,0x08,0x08,0x00 };

// -------------------- DS1302 helpers --------------------

static inline u8 bcd2bin_u8(u8 bcd) { return ((bcd >> 4) * 10) + (bcd & 0x0F); }
static inline u8 bin2bcd_u8(u8 v)   { return ((v / 10) << 4) | (v % 10); }

static inline void ds_ce(int v)  { gpio_set_value(ds_ce_gpio, v); }
static inline void ds_clk(int v) { gpio_set_value(ds_clk_gpio, v); }

static inline void ds_dat_out(int v) { gpio_direction_output(ds_dat_gpio, v); }
static inline void ds_dat_in(void)   { gpio_direction_input(ds_dat_gpio); }
static inline int  ds_dat_read(void) { return gpio_get_value(ds_dat_gpio); }

static void ds1302_start(void)
{
    ds_clk(0);
    ds_ce(1);
    udelay(4);
}

static void ds1302_stop(void)
{
    ds_ce(0);
    udelay(4);
}

static void ds1302_write_byte(u8 val)
{
    int i;
    for (i = 0; i < 8; i++) {
        ds_dat_out(val & 0x01);
        udelay(1);
        ds_clk(1);
        udelay(1);
        ds_clk(0);
        udelay(1);
        val >>= 1;
    }
}

static u8 ds1302_read_byte(void)
{
    int i;
    u8 val = 0;

    ds_dat_in();
    for (i = 0; i < 8; i++) {
        ds_clk(1);
        udelay(1);
        if (ds_dat_read())
            val |= (1 << i);
        ds_clk(0);
        udelay(1);
    }
    return val;
}

static void ds1302_write_reg(u8 addr_write, u8 data)
{
    ds1302_start();
    ds1302_write_byte(addr_write); // write address (LSB=0)
    ds1302_write_byte(data);
    ds1302_stop();
}

static int ds1302_read_time(struct ds_time *t)
{
    u8 raw[8];
    int i;

    ds1302_start();
    ds1302_write_byte(0xBF); // Clock Burst Read
    for (i = 0; i < 8; i++)
        raw[i] = ds1302_read_byte();
    ds1302_stop();

    t->sec  = bcd2bin_u8(raw[0] & 0x7F);
    t->min  = bcd2bin_u8(raw[1] & 0x7F);

    if (raw[2] & 0x80) {
        // 12h 모드(간단 처리)
        t->hour = bcd2bin_u8(raw[2] & 0x1F);
    } else {
        t->hour = bcd2bin_u8(raw[2] & 0x3F);
    }

    t->mday = bcd2bin_u8(raw[3] & 0x3F);
    t->mon  = bcd2bin_u8(raw[4] & 0x1F);
    t->wday = bcd2bin_u8(raw[5] & 0x07);
    t->year = bcd2bin_u8(raw[6]);

    if (t->sec > 59 || t->min > 59 || t->hour > 23)
        return -EINVAL;
    return 0;
}

static int ds1302_set_datetime(const struct ds_time *t)
{
    // WP off
    ds1302_write_reg(0x8E, 0x00);

    // CH(bit7)=0
    ds1302_write_reg(0x80, bin2bcd_u8(t->sec)  & 0x7F);
    ds1302_write_reg(0x82, bin2bcd_u8(t->min)  & 0x7F);
    ds1302_write_reg(0x84, bin2bcd_u8(t->hour) & 0x3F);
    ds1302_write_reg(0x86, bin2bcd_u8(t->mday) & 0x3F);
    ds1302_write_reg(0x88, bin2bcd_u8(t->mon)  & 0x1F);
    ds1302_write_reg(0x8A, bin2bcd_u8(t->wday) & 0x07);
    ds1302_write_reg(0x8C, bin2bcd_u8(t->year));

    // WP on (선택)
    ds1302_write_reg(0x8E, 0x80);

    return 0;
}

// -------------------- parse YYYYMMDDhhmmss --------------------
static inline bool is_digit(char c) { return c >= '0' && c <= '9'; }

static int parse_2d(const char *s, u8 *out)
{
    if (!is_digit(s[0]) || !is_digit(s[1])) return -EINVAL;
    *out = (s[0]-'0')*10 + (s[1]-'0');
    return 0;
}

static int parse_4d_year(const char *s, int *year)
{
    int i, y = 0;
    for (i = 0; i < 4; i++) {
        if (!is_digit(s[i])) return -EINVAL;
        y = y*10 + (s[i]-'0');
    }
    *year = y;
    return 0;
}

static int parse_datetime_14(const char *s14, struct ds_time *t)
{
    int ret, year4;
    u8 mon, mday, hour, min, sec;

    ret = parse_4d_year(s14 + 0, &year4); if (ret) return ret;
    ret = parse_2d(s14 + 4, &mon);        if (ret) return ret;
    ret = parse_2d(s14 + 6, &mday);       if (ret) return ret;
    ret = parse_2d(s14 + 8, &hour);       if (ret) return ret;
    ret = parse_2d(s14 +10, &min);        if (ret) return ret;
    ret = parse_2d(s14 +12, &sec);        if (ret) return ret;

    if (year4 < 2000 || year4 > 2099) return -ERANGE;
    if (mon < 1 || mon > 12) return -ERANGE;
    if (mday < 1 || mday > 31) return -ERANGE;
    if (hour > 23 || min > 59 || sec > 59) return -ERANGE;

    t->year = (u8)(year4 - 2000);
    t->mon  = mon;
    t->mday = mday;
    t->hour = hour;
    t->min  = min;
    t->sec  = sec;
    t->wday = 1; // 요일 입력 없으면 임시(필요하면 나중에 계산해서 넣기)
    return 0;
}

// -------------------- I2C SSD1306 helpers --------------------
static int oled_i2c_create_client(void)
{
    struct i2c_board_info info = { I2C_BOARD_INFO("ssd1306", 0) };
    info.addr = i2c_addr;

    oled_adap = i2c_get_adapter(i2c_bus);
    if (!oled_adap)
        return -ENODEV;

    oled_i2c = i2c_new_client_device(oled_adap, &info); //8번 버스 객체에 0x3c 장치(client)를 만들었다.
    if (IS_ERR(oled_i2c)) {
        int ret = PTR_ERR(oled_i2c);
        oled_i2c = NULL;
        i2c_put_adapter(oled_adap);
        oled_adap = NULL;
        return ret;
    }
    return 0;
}

static void oled_i2c_destroy_client(void)
{
    if (oled_i2c) {
        i2c_unregister_device(oled_i2c);
        oled_i2c = NULL;
    }
    if (oled_adap) {
        i2c_put_adapter(oled_adap);
        oled_adap = NULL;
    }
}

// control byte: cmd=0x00, data=0x40
static int oled_i2c_write(bool is_data, const u8 *buf, size_t len)
{
    u8 tmp[1 + 16];
    size_t off = 0;
    int ret;

    if (!oled_i2c)
        return -ENODEV;

    tmp[0] = is_data ? 0x40 : 0x00;// command면 0x00 data면 0x01

    while (off < len) {
        size_t n = min((size_t)16, len - off);
        memcpy(&tmp[1], &buf[off], n);//tmp[1]에 buf[off]를 n바이트 만큼 복사
        ret = i2c_master_send(oled_i2c, tmp, 1 + n); //oled_12c에 tmp의 주소값을 통해 
        // 1+n만큼의 바이트를 보냄?.client로
        if (ret < 0) return ret;
        if (ret != 1 + n) return -EIO;
        off += n;
    }
    return 0;
}

static inline int oled_cmd(u8 c) { return oled_i2c_write(false, &c, 1); }
static inline int oled_data(const u8 *p, size_t n) { return oled_i2c_write(true, p, n); }

static void fb_clear(void) { memset(fb, 0x00, sizeof(fb)); }

static void fb_draw_char6x8(int x, int page, char c)
{
    const u8 *g = NULL;
    int i, idx;

    if (c >= '0' && c <= '9') {
        idx = c - '0';
        g = font6x8[idx];
    } else if (c == ':') {
        g = font6x8[10];
    } else if (c == '-') {
        g = glyph_dash;
    } else if (c == ' ') {
        g = glyph_space;
    } else if (c == 'T') {
        g = glyph_T;
     } else if (c == 'S') {
    g = glyph_S;
    } else if (c == 'E') {
    g = glyph_E;   
    } else if (c == 'H') {
        g = glyph_H;
    } else if (c == 'C') {
        g = glyph_C;
    } else if (c == '%') {
        g = glyph_percent;
    } else {
        return;
    }

    if (page < 0 || page >= 8) return;
    if (x < 0 || x + 6 > OLED_W) return;

    for (i = 0; i < 6; i++)
        fb[page * OLED_W + x + i] = g[i];
}


static void fb_draw_str6x8(int x, int page, const char *s)
{
    while (*s) {
        fb_draw_char6x8(x, page, *s++);
        x += 6;
        if (x >= OLED_W) break;
    }
}

static int oled_init(void)
{
    // RST 핀 없음 -> reset pulse 없음

    // SSD1306 init (128x64, charge pump on)
    oled_cmd(0xAE);                // display off
    oled_cmd(0xD5); oled_cmd(0x80);
    oled_cmd(0xA8); oled_cmd(0x3F);
    oled_cmd(0xD3); oled_cmd(0x00);
    oled_cmd(0x40);
    oled_cmd(0x8D); oled_cmd(0x14); // charge pump on
    oled_cmd(0x20); oled_cmd(0x00); // horizontal addressing
    oled_cmd(0xA1);
    oled_cmd(0xC8);
    oled_cmd(0xDA); oled_cmd(0x12);
    oled_cmd(0x81); oled_cmd(0xCF);
    oled_cmd(0xD9); oled_cmd(0xF1);
    oled_cmd(0xDB); oled_cmd(0x40);
    oled_cmd(0xA4);
    oled_cmd(0xA6);
    oled_cmd(0xAF);                // display on
    return 0;
}

static void oled_flush(void)
{
    int page;
    for (page = 0; page < 8; page++) {
        oled_cmd(0xB0 | page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        oled_data(&fb[page * OLED_W], OLED_W);
    }
}

// -------------------- tick work (1s) --------------------
static void tick_fn(struct work_struct *work)
{   
    int ev;
    int guard = 8;

    char buf_th[16];      // "T25C H60%"
    char buf_dt[24];      // "2025-12-17"
    char buf_tm[16];      // "17:40:00"
    int year4;

    /* =========================
     * 1) 로터리 이벤트: 즉시 반영
     * ========================= */
  while (guard-- > 0 && (ev = rotary_get_event()) != ROT_EV_NONE) {

    if (ev == ROT_EV_BTN_DOWN) {
            if (g_mode == UI_NORMAL) {
                enter_set_mode();
            } else { // UI_SET
                if (g_field == FLD_SEC) {          
                    save_and_exit_set_mode();
                } else {
                    field_next();
                }
            }
            continue;
        }

        /* 회전은 SET 모드에서만 편집 */
        if (g_mode == UI_SET) {
            if (ev == ROT_EV_CW)  edit_add(&g_edit, +1);
            if (ev == ROT_EV_CCW) edit_add(&g_edit, -1);
        }
    }

    /* =========================
     * 2) 센서/RTC 캐시: 1초마다만
     * ========================= */
   if (jiffies - last_sense_j >= msecs_to_jiffies(SENSE_TICK_MS)) {
    last_sense_j = jiffies;
        int ret;
        pr_info("DHT: try read (j=%lu)\n", jiffies);

        ret = dht11_read_values(&temp_cache, &humi_cache);
        if (ret) {
            pr_info("DHT: read FAIL ret=%d\n", ret);
            temp_cache = -1;
            humi_cache = -1;
        } else {
            pr_info("DHT: read OK t=%d h=%d\n", temp_cache, humi_cache);
        }

    if (g_mode == UI_NORMAL) {
        mutex_lock(&ds_lock);
        cache_ok = (ds1302_read_time(&t_cache) == 0);
        mutex_unlock(&ds_lock);
    }
}

    /* =========================
     * 3) 커서 깜빡임: 1초 토글(SET에서만)
     * ========================= */
    if (g_mode == UI_SET) {
        if (time_after(jiffies, last_blink_j + msecs_to_jiffies(BLINK_MS))) {
            last_blink_j = jiffies;
            g_blink_on = !g_blink_on;
        }
    } else {
        g_blink_on = true; // NORMAL에선 마스크 안 쓰게 항상 ON 처리
    }

    /* =========================
     * 4) 화면에 출력할 문자열 만들기
     * ========================= */

    /* (A) 온습도: 캐시값만 사용 -> 안 깜빡임 */
    if (temp_cache >= 0 && humi_cache >= 0)
        snprintf(buf_th, sizeof(buf_th), "T%02dC H%02d%%", temp_cache, humi_cache);
    else
        snprintf(buf_th, sizeof(buf_th), "T--C H--%%");

    /* (B) 날짜/시간: SET이면 g_edit, NORMAL이면 t_cache */
    if (g_mode == UI_SET) {
        year4 = 2000 + g_edit.year;
        snprintf(buf_dt, sizeof(buf_dt), "%04d-%02u-%02u", year4, g_edit.mon, g_edit.mday);
        snprintf(buf_tm, sizeof(buf_tm), "%02u:%02u:%02u", g_edit.hour, g_edit.min, g_edit.sec);

        /* blink는 표시만 가리기(값 변경과 무관) */
        apply_blink_mask(buf_dt, buf_tm, g_blink_on);

    } else {
        if (cache_ok) {
            year4 = 2000 + t_cache.year;
            snprintf(buf_dt, sizeof(buf_dt), "%04d-%02u-%02u", year4, t_cache.mon, t_cache.mday);
            snprintf(buf_tm, sizeof(buf_tm), "%02u:%02u:%02u", t_cache.hour, t_cache.min, t_cache.sec);
        } else {
            snprintf(buf_dt, sizeof(buf_dt), "---- -- --");
            snprintf(buf_tm, sizeof(buf_tm), "--:--:--");
        }
    }

    /* =========================
     * 5) OLED draw
     * ========================= */
    fb_clear();
    if (g_mode == UI_SET)
    fb_draw_str6x8(0, 0, "SET");
    fb_draw_str6x8(74, 0, buf_th);
    fb_draw_str6x8(0, 2, buf_dt);
    fb_draw_str6x8(0, 4, buf_tm);
    oled_flush();

    /* =========================
     * 6) 다음 UI tick: 50ms
     * ========================= */
    mod_delayed_work(system_wq, &tick_work, msecs_to_jiffies(UI_TICK_MS));
}


// -------------------- char device fops --------------------
static int my_open(struct inode *inode, struct file *file) { return 0; }
static int my_release(struct inode *inode, struct file *file) { return 0; }

static ssize_t my_read(struct file *file, char __user *ubuf, size_t count, loff_t *ppos)
{
    struct ds_time t;
    char out[32];
    int len, ret, year4;

    if (*ppos > 0) return 0; // EOF

    mutex_lock(&ds_lock);
    ret = ds1302_read_time(&t);
    mutex_unlock(&ds_lock);
    if (ret) return ret;

    year4 = 2000 + t.year;
    len = scnprintf(out, sizeof(out), "%04d%02u%02u%02u%02u%02u\n",
                    year4, t.mon, t.mday, t.hour, t.min, t.sec);

    if (count < len) return -EINVAL;
    if (copy_to_user(ubuf, out, len)) return -EFAULT;

    *ppos += len;
    return len;
}

static ssize_t my_write(struct file *file, const char __user *ubuf, size_t count, loff_t *ppos)
{
    char kbuf[32];
    size_t n;
    struct ds_time t;
    int ret;

    if (count == 0) return 0;

    n = min(count, sizeof(kbuf) - 1);
    if (copy_from_user(kbuf, ubuf, n)) return -EFAULT;
    kbuf[n] = '\0';
    strim(kbuf);

    if (strlen(kbuf) != 14) return -EINVAL;

    ret = parse_datetime_14(kbuf, &t);
    if (ret) return ret;

    mutex_lock(&ds_lock);
    ret = ds1302_set_datetime(&t);
    mutex_unlock(&ds_lock);
    if (ret) return ret;

    return count;
}

static const struct file_operations fops = {
    .owner   = THIS_MODULE,
    .open    = my_open,
    .read    = my_read,
    .write   = my_write,
    .release = my_release,
};

// -------------------- module init/exit --------------------
static int __init ds1302_oled_init(void)
{
    int ret;
    struct ds_time t;

    pr_info("=== ds1302_oled init (i2c=%d addr=0x%x) ===\n", i2c_bus, i2c_addr);

    // 1) chrdev
    ret = alloc_chrdev_region(&dev_num, 0, 1, DRIVER_NAME);
    if (ret) return ret;

    cdev_init(&ds_cdev, &fops);
    ret = cdev_add(&ds_cdev, dev_num, 1);
    if (ret) goto err_chr;

    ds_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ds_class)) {
        ret = PTR_ERR(ds_class);
        ds_class = NULL;
        goto err_cdev;
    }

    ds_dev = device_create(ds_class, NULL, dev_num, NULL, DRIVER_NAME);
    if (IS_ERR(ds_dev)) {
        ret = PTR_ERR(ds_dev);
        ds_dev = NULL;
        goto err_class;
    }

    // 2) GPIO for DS1302
    ret = gpio_request(ds_ce_gpio, "ds1302_ce");
    if (ret) goto err_dev;
    ret = gpio_request(ds_clk_gpio, "ds1302_clk");
    if (ret) goto err_gpio1;
    ret = gpio_request(ds_dat_gpio, "ds1302_dat");
    if (ret) goto err_gpio2;

    gpio_direction_output(ds_ce_gpio, 0);
    gpio_direction_output(ds_clk_gpio, 0);
    gpio_direction_output(ds_dat_gpio, 0);

    // 3) I2C client
    ret = oled_i2c_create_client();
    if (ret) {
        pr_err("cannot create I2C client (bus=%d addr=0x%x)\n", i2c_bus, i2c_addr);
        goto err_gpio3;
    }

    // 4) OLED init + clear
    ret = oled_init();
    if (ret) {
        pr_err("oled_init failed: %d\n", ret);
        goto err_i2c;
    }
    fb_clear();
    oled_flush();

    // 5) (optional) init datetime set
    if (init_datetime && strlen(init_datetime) == 14) {
        ret = parse_datetime_14(init_datetime, &t);
        if (!ret) {
            mutex_lock(&ds_lock);
            ds1302_set_datetime(&t);
            mutex_unlock(&ds_lock);
            pr_info("init_datetime applied: %s\n", init_datetime);
        } else {
            pr_err("init_datetime invalid: %s\n", init_datetime);
        }
    }
    last_sense_j = jiffies - msecs_to_jiffies(SENSE_TICK_MS);

    // 6) start tick
    INIT_DELAYED_WORK(&tick_work, tick_fn);
    schedule_delayed_work(&tick_work, HZ);

    pr_info("ds1302_oled started: /dev/%s\n", DRIVER_NAME);
    return 0;

err_i2c:
    oled_i2c_destroy_client();
err_gpio3:
    gpio_free(ds_dat_gpio);
err_gpio2:
    gpio_free(ds_clk_gpio);
err_gpio1:
    gpio_free(ds_ce_gpio);
err_dev:
    if (ds_dev) { device_destroy(ds_class, dev_num); ds_dev = NULL; }
err_class:
    if (ds_class) { class_destroy(ds_class); ds_class = NULL; }
err_cdev:
    cdev_del(&ds_cdev);
err_chr:
    unregister_chrdev_region(dev_num, 1);
    return ret;
}

static void __exit ds1302_oled_exit(void)
{
    cancel_delayed_work_sync(&tick_work);

    fb_clear();
    oled_flush();

    oled_i2c_destroy_client();

    gpio_free(ds_dat_gpio);
    gpio_free(ds_clk_gpio);
    gpio_free(ds_ce_gpio);

    if (ds_dev) { device_destroy(ds_class, dev_num); ds_dev = NULL; }
    if (ds_class) { class_destroy(ds_class); ds_class = NULL; }

    cdev_del(&ds_cdev);
    unregister_chrdev_region(dev_num, 1);

    pr_info("ds1302_oled exit\n");
}

module_init(ds1302_oled_init);
module_exit(ds1302_oled_exit);
