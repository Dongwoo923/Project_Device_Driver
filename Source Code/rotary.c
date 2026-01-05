#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/spinlock.h>
#include <linux/export.h>
#include <linux/irq.h>   
#include <linux/types.h> 

#define DRIVER_NAME "rotary_device_driver"
#define CLASS_NAME "rotary_device_class"
#define S1_GPIO 23
#define S2_GPIO 24
#define SW_GPIO 25 
#define DEBOUNCE_6MS 2 // debounce time 2ms 
#define SW_DEBOUNCE_MS 30 // sw debounce time 20ms 
#define STEPS_PER_DETENT 2      // 보통 4, 필요시 2로 튜닝
#define ROT_GLITCH_MS    1      // 글리치 컷(1~2ms 추천)

/* 메타 정보 */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("kkk");
MODULE_DESCRIPTION("rotary driver");
static bool rot_irq_enabled = true; 
static dev_t device_number;
static struct cdev rotary_cdev;
static struct class *rotary_class;
static int interrupt_num_sw;
static int interrupt_num_s1;	// int number of s1 gpio
static int interrupt_num_s2; 
static unsigned long last_interrupt_time_sw = 0;
static int data_update_finish=0; // data ready

enum { EVT_NONE=0, EVT_ROT=1, EVT_BTN=2 };
static int last_evt_type = EVT_NONE;
static long last_evt_value = 0;  // ROT: +1/-1, BTN: 1
static int btn_latched = 0;
static u8 prev_ab;              // 이전 AB 상태
static int step_acc;            // 전이 누적
static DECLARE_WAIT_QUEUE_HEAD(rotary_wait_queue);
static inline u8 read_ab(void)
{
    u8 a = gpio_get_value(S1_GPIO) ? 1 : 0;
    u8 b = gpio_get_value(S2_GPIO) ? 1 : 0;
    return (a << 1) | b;
}

/* 유효 전이만 인정 */
static inline int decode_step(u8 from, u8 to)
{
    switch ((from << 2) | to) {
    /* CW: 00->01->11->10->00 */
    case 0b0001: case 0b0111: case 0b1110: case 0b1000:
        return +1;
    /* CCW: 00->10->11->01->00 */
    case 0b0010: case 0b1011: case 0b1101: case 0b0100:
        return -1;
    default:
        return 0;  // invalid (bounce/glitch)
    }
}
void rotary_irq_enable(bool on)
{   pr_info("ROT IRQ %s\n", on ? "ON" : "OFF");
    /* 중복 호출 방지 */
    if (on) {
        if (!rot_irq_enabled) {
            enable_irq(interrupt_num_s1);
            enable_irq(interrupt_num_s2);
            rot_irq_enabled = true;
        }
    } else {
        if (rot_irq_enabled) {
            disable_irq(interrupt_num_s1);
            disable_irq(interrupt_num_s2);
            rot_irq_enabled = false;
        }
    }
}
EXPORT_SYMBOL_GPL(rotary_irq_enable);
enum rotary_evt_type {
    ROT_EV_NONE = 0,
    ROT_EV_CW,
    ROT_EV_CCW,
    ROT_EV_BTN_DOWN,
    ROT_EV_BTN_UP,
};

static DEFINE_SPINLOCK(rot_lock);
static int rot_last_evt = ROT_EV_NONE;

/* 디코딩 결과가 나오면 여기로 넣어 */
static inline void rot_push_evt(int ev)
{
    unsigned long flags;
    spin_lock_irqsave(&rot_lock, flags);
    rot_last_evt = ev;            // 가장 단순: 마지막 이벤트 1개만 유지
    spin_unlock_irqrestore(&rot_lock, flags);
}

/* ds1302_oled에서 가져다 쓸 함수 */
int rotary_get_event(void)
{
    unsigned long flags;
    int ev;

    spin_lock_irqsave(&rot_lock, flags);
    ev = rot_last_evt;
    rot_last_evt = ROT_EV_NONE;   // 읽으면 소진
    spin_unlock_irqrestore(&rot_lock, flags);

    return ev;
}
EXPORT_SYMBOL_GPL(rotary_get_event);
// ---- interrupt handler
static irqreturn_t rotary_ab_int_handler(int irq, void *dev_id)
{
    unsigned long now = jiffies;

    /* 아주 짧은 글리치 컷 */
    if (time_before(now, last_irq_ab + msecs_to_jiffies(ROT_GLITCH_MS)))
        return IRQ_HANDLED;
    last_irq_ab = now;

    u8 ab = read_ab();
    int s = decode_step(prev_ab, ab);
    prev_ab = ab;

    if (s) {
        step_acc += s;

        if (step_acc >= STEPS_PER_DETENT) {
            step_acc = 0;

            last_evt_type = EVT_ROT;
            last_evt_value = +1;
            rot_push_evt(ROT_EV_CW);

            data_update_finish = 1;
            wake_up_interruptible(&rotary_wait_queue);
        } else if (step_acc <= -STEPS_PER_DETENT) {
            step_acc = 0;

            last_evt_type = EVT_ROT;
            last_evt_value = -1;
            rot_push_evt(ROT_EV_CCW);

            data_update_finish = 1;
            wake_up_interruptible(&rotary_wait_queue);
        }
    }

    return IRQ_HANDLED;
}

static irqreturn_t sw_int_handler(int irq, void *dev_id)
{
    unsigned long now = jiffies;
    unsigned long dj  = msecs_to_jiffies(SW_DEBOUNCE_MS);
    int v = gpio_get_value(SW_GPIO);  // 0: pressed, 1: released (active-low)

    /* ---------- UP은 debounce 없이 latch만 즉시 해제 ---------- */
    if (v == 1 && btn_latched) {
        btn_latched = 0;
        return IRQ_HANDLED;
    }

    /* ---------- DOWN만 debounce ---------- */
    if (time_before(now, last_interrupt_time_sw + dj))
        return IRQ_HANDLED;
    last_interrupt_time_sw = now;

    if (v == 0 && !btn_latched) {
        btn_latched = 1;

        rot_push_evt(ROT_EV_BTN_DOWN);

        last_evt_type  = EVT_BTN;
        last_evt_value = 1;

        data_update_finish = 1;
        wake_up_interruptible(&rotary_wait_queue);
        return IRQ_HANDLED;
    }

    return IRQ_HANDLED;
}

static ssize_t rotary_read(struct file *file, char __user *user_buff, size_t count, loff_t *ppos)
{
	char buffer[64];
	int len;
	// blocking i/o: wait while update data 
	wait_event_interruptible(rotary_wait_queue, data_update_finish != 0);
	data_update_finish=0;
	if (last_evt_type == EVT_ROT) {
        len = snprintf(buffer, sizeof(buffer),
                       "ROT %ld\n", last_evt_value);
    }
    else if (last_evt_type == EVT_BTN) {
        len = snprintf(buffer, sizeof(buffer),
                       "BTN\n");
    }
    else {
        len = snprintf(buffer, sizeof(buffer),
                       "NONE\n");
    }

    /* 다음 이벤트를 위해 초기화 */
    last_evt_type = EVT_NONE;

	// copy user space
	if(copy_to_user(user_buff, buffer, len))
		return -EFAULT;
	
	return len;

}
static struct file_operations fops = {
	.owner = THIS_MODULE,
	.read  = rotary_read
};

static int __init  rotary_driver_init(void)
{
	int ret;
	printk(KERN_INFO "=== rotary initializing ======\n");
	// 1. alloc device number 
	if ((ret = alloc_chrdev_region(&device_number ,0 ,1 ,DRIVER_NAME )) == -1){
		printk(KERN_ERR "ERROR: alloc_chrdev_regin .......\n");
		return ret;
	}
	// 2. register char device
	cdev_init(&rotary_cdev, &fops);
	if ((ret = cdev_add(&rotary_cdev, device_number,1)) == -1){
		printk(KERN_ERR "ERROR: cdev_add .......\n");
		unregister_chrdev_region(device_number, 1);
		return ret;
	}
	// 3. create class & create device /dev/rotary_driver
	rotary_class = class_create(THIS_MODULE, CLASS_NAME);
	if(IS_ERR(rotary_class))
	{
		cdev_del(&rotary_cdev);
		unregister_chrdev_region(device_number, 1);
		return PTR_ERR(rotary_class);
	}
	device_create(rotary_class, NULL, device_number, NULL, DRIVER_NAME);
	// 4. request gpio
	if(gpio_request(S1_GPIO,"my_rotary") || gpio_request(S2_GPIO, "my_rotary") ||
		gpio_request(SW_GPIO,"my_rotary_sw")) 
	{
		printk(KERN_ERR "ERROR: gpio_request......\n");
		return -1;
	}
	// set input mode 
	gpio_direction_input(S1_GPIO);
    gpio_direction_input(S2_GPIO);
	gpio_direction_input(SW_GPIO);
	// 5. assign gpio to irq
prev_ab = read_ab();
step_acc = 0;
last_irq_ab = 0;

/* S1 IRQ */
interrupt_num_s1 = gpio_to_irq(S1_GPIO);
ret = request_irq(interrupt_num_s1, rotary_ab_int_handler,
                 IRQF_TRIGGER_FALLING,
                  "my_rotary_irq_s1", NULL);
if (ret) return ret;

/* S2 IRQ */
interrupt_num_s2 = gpio_to_irq(S2_GPIO);
ret = request_irq(interrupt_num_s2, rotary_ab_int_handler,
                 IRQF_TRIGGER_FALLING,
                  "my_rotary_irq_s2", NULL);
/* SW IRQ */
interrupt_num_sw = gpio_to_irq(SW_GPIO);
ret = request_irq(interrupt_num_sw, sw_int_handler,
                  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                  "my_rotary_irq_sw", NULL);
if (ret){
    free_irq(interrupt_num_s1, NULL);
    return ret;
}
rotary_irq_enable(false);
printk(KERN_INFO "rotary driver init success......\n");
return 0;
}

static void __exit rotary_driver_exit(void)
{
	free_irq(interrupt_num_s1, NULL);
    free_irq(interrupt_num_s2, NULL); 
	free_irq(interrupt_num_sw, NULL);
	gpio_free(S1_GPIO);
	gpio_free(S2_GPIO);
	gpio_free(SW_GPIO);
	device_destroy(rotary_class, device_number);
	class_destroy(rotary_class);
	cdev_del(&rotary_cdev);
	unregister_chrdev_region(device_number, 1);
	printk(KERN_INFO "rotary_driver_exit !!!!!\n");

}

module_init(rotary_driver_init);
module_exit(rotary_driver_exit);
