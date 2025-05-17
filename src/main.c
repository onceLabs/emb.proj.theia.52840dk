#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define IMX219_REG_SOFT_RESET  0x0103
#define IMX219_REG_MODEL_ID    0x0000
#define IMX219_I2C_ADDR        0x10 

static const struct gpio_dt_spec pwr = {
  .port =     DEVICE_DT_GET(DT_NODELABEL(gpio1)),
  .pin = 4,
  .dt_flags = GPIO_OUTPUT_INACTIVE,
};
static const struct gpio_dt_spec xclk = {
  .port =     DEVICE_DT_GET(DT_NODELABEL(gpio1)),
  .pin = 5,
  .dt_flags = GPIO_OUTPUT_INACTIVE,
};
static const struct i2c_dt_spec cam = {
  .bus  = DEVICE_DT_GET(DT_NODELABEL(i2c0)),
  .addr = IMX219_I2C_ADDR,
};



static int imx219_write_u8(uint16_t reg, uint8_t val)
{
  uint8_t buf[3] = { reg >> 8, reg & 0xFF, val };
  return i2c_write_dt(&cam, buf, sizeof(buf));
}

static int imx219_read_u16(uint16_t reg, uint16_t *out)
{
  uint8_t addr[2] = { reg >> 8, reg & 0xFF };
  uint8_t data[2];
  int err = i2c_write_read_dt(&cam, addr, sizeof(addr), data, sizeof(data));
  if (!err) {
    *out = (data[0] << 8) | data[1];
  }
  return err;
}


int main(void)
{
  if (!device_is_ready(pwr.port) || !device_is_ready(cam.bus)) {
    printk("Required device not ready\n");
    return;
  }

  const uint32_t i2c_cfg =
  I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;

  int err = i2c_configure(cam.bus, i2c_cfg);
  if (err) {
    printk("I2C configure failed: %d\n", err);
    return;
  }

  gpio_pin_configure_dt(&pwr, GPIO_OUTPUT_LOW);
  gpio_pin_configure_dt(&xclk, GPIO_OUTPUT_LOW);
  gpio_pin_set_dt(&pwr, 0);
  gpio_pin_set_dt(&xclk, 0);
  k_msleep(100);                 
  gpio_pin_set_dt(&pwr, 0);
  gpio_pin_set_dt(&xclk, 1);

  k_msleep(20);


  
  int ret;

  ret = imx219_write_u8(IMX219_REG_SOFT_RESET, 0x01);
  if (ret) {
    printk("Failed to write soft reset: %d\n", ret);
    return;
  }
  printk("Soft reset sent, waiting 1s …\n");
  k_msleep(1000);

  uint16_t id;
  if (imx219_read_u16(IMX219_REG_MODEL_ID, &id)) {
    printk("Model ID read failed\n");
    return;
  }
  printk("IMX219 MODEL_ID = 0x%04X (expected 0x0219)\n", id);
}
