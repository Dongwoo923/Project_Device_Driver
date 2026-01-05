#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#define DRIVER_NAME   "dht11"
#define CLASS_NAME    "dht11_class"
#define GPIO_PIN       4

MODULE_LICENSE("GPL");
MODULE_AUTHOR("kkk");
MODULE_DESCRIPTION("DHT11 driver");

static DEFINE_MUTEX(dht_lock);
static struct class *dht11_class=NULL;
static dev_t dev_num;
static struct cdev dht11_cdev;
static int read_dht11(int *temp, int *humi);
int dht11_read_values(int *temp, int *humi);

int dht11_read_values(int *temp, int *humi)
{
    int ret;
    if (!temp || !humi) return -EINVAL;

    mutex_lock(&dht_lock);
    ret = read_dht11(temp, humi);
    mutex_unlock(&dht_lock);

    return ret;
}
EXPORT_SYMBOL_GPL(dht11_read_values);

static int wait_pin_status(int level, int time)
{
	int counter =0;

	while(gpio_get_value(GPIO_PIN) != level) {
		if (++counter > time) return -1;
		udelay(1);
	}
	return counter;
}
static ssize_t dht11_dev_read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	int temp=0, humi = 0;
	int ret;
	char msg_buff[80];

	if (*offset >0)
	{

		*offset = 0;
		return 0;

	}

	ret = read_dht11(&temp, &humi);
	if (ret == 0)
		sprintf(msg_buff, "temp: %d c humi: %d %%\n", temp, humi);
	else sprintf(msg_buff, "DHT11 read error !!!! %d\n", ret);

	if (copy_to_user(buffer, msg_buff, strlen(msg_buff)))
		return -1;
	*offset = strlen(msg_buff);

	return strlen(msg_buff);
}
static int read_dht11(int *temp, int *humi)
{
    unsigned char data[5] = {0};
    unsigned long flags;
    int i, ret = 0;

    gpio_direction_output(GPIO_PIN, 0);
    msleep(20);
    gpio_set_value(GPIO_PIN, 1);
    udelay(30);

    gpio_direction_input(GPIO_PIN);
    udelay(2);

    preempt_disable();
    local_irq_save(flags);

    if (wait_pin_status(0, 250) < 0) { ret = -ETIMEDOUT; goto out; }
    if (wait_pin_status(1, 250) < 0) { ret = -ETIMEDOUT; goto out; }
    if (wait_pin_status(0, 250) < 0) { ret = -ETIMEDOUT; goto out; }

    for (i = 0; i < 40; i++) {
        if (wait_pin_status(1, 250) < 0) { ret = -ETIMEDOUT; goto out; }
        udelay(35);
        if (gpio_get_value(GPIO_PIN))
            data[i/8] |= (1 << (7 - (i % 8)));
        // ✅ 0/1 상관없이 항상 LOW까지 기다림
        if (wait_pin_status(0, 250) < 0) { ret = -ETIMEDOUT; goto out; }
    }

out:
    local_irq_restore(flags);
    preempt_enable();

    if (ret) return ret;

    if (data[4] != ((data[0]+data[1]+data[2]+data[3]) & 0xFF))
        return -EIO;

    *humi = data[0];
    *temp = data[2];
    return 0;
}


static const struct file_operations fops = {
    .read  = dht11_dev_read
};

static int __init dht11_driver_init(void)
{
    int ret;

    printk(KERN_INFO "=== dht11 initializing ====\n");

    /* 1. 문자 디바이스 번호 할당 */
    ret = alloc_chrdev_region(&dev_num, 0, 1, DRIVER_NAME);
    if (ret < 0) {
        printk(KERN_ERR "ERROR: alloc_chrdev_region\n");
        return ret;
    }

    /* 2. cdev 등록 */
    cdev_init(&dht11_cdev, &fops);
    dht11_cdev.owner = THIS_MODULE;

    ret = cdev_add(&dht11_cdev, dev_num, 1);
    if (ret < 0) {
        printk(KERN_ERR "ERROR: cdev_add\n");
        unregister_chrdev_region(dev_num, 1);
        return ret;
    }

    /* 3. class / device 생성 (udev용) */
    dht11_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(dht11_class)) {
        printk(KERN_ERR "ERROR: class_create\n");
        cdev_del(&dht11_cdev);
        unregister_chrdev_region(dev_num, 1);
        return PTR_ERR(dht11_class);
    }

    if (IS_ERR(device_create(dht11_class, NULL, dev_num,
                             NULL, DRIVER_NAME))) {
        printk(KERN_ERR "ERROR: device_create\n");
        class_destroy(dht11_class);
        cdev_del(&dht11_cdev);
        unregister_chrdev_region(dev_num, 1);
        return -ENOMEM;
    }

    /* 4. GPIO 요청 및 설정 */
    ret = gpio_request( GPIO_PIN, "my_DHT11_data_pin");
    if (ret) {
        printk(KERN_ERR "ERROR: gpio_request \n");
        return -1;
    }

    printk(KERN_INFO "dth11 driver init success\n");
    return 0;
}

static void __exit dht11_driver_exit(void)
{
    gpio_free(GPIO_PIN);
    device_destroy(dht11_class, dev_num);
    class_destroy(dht11_class);
    cdev_del(&dht11_cdev);
    unregister_chrdev_region(dev_num, 1);
    printk(KERN_INFO "DHT11_driver_exit !!!!\n");
}

module_init(dht11_driver_init);
module_exit(dht11_driver_exit);
