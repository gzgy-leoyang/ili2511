#include <linux/module.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

MODULE_INFO(vermagic, VERMAGIC_STRING);

__visible struct module __this_module
__attribute__((section(".gnu.linkonce.this_module"))) = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

MODULE_INFO(intree, "Y");

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0x75193b8b, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x51eafc8e, __VMLINUX_SYMBOL_STR(param_ops_int) },
	{ 0xd9860367, __VMLINUX_SYMBOL_STR(i2c_del_driver) },
	{ 0x86ee9077, __VMLINUX_SYMBOL_STR(i2c_register_driver) },
	{ 0xfa2a45e, __VMLINUX_SYMBOL_STR(__memzero) },
	{ 0x3a3bdafd, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0xdaccb431, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x43a53735, __VMLINUX_SYMBOL_STR(__alloc_workqueue_key) },
	{ 0x8d9560f7, __VMLINUX_SYMBOL_STR(input_register_device) },
	{ 0xa3f100fe, __VMLINUX_SYMBOL_STR(input_set_abs_params) },
	{ 0x676bbc0f, __VMLINUX_SYMBOL_STR(_set_bit) },
	{ 0xf615bafc, __VMLINUX_SYMBOL_STR(input_allocate_device) },
	{ 0xf9a482f9, __VMLINUX_SYMBOL_STR(msleep) },
	{ 0x5d2f2370, __VMLINUX_SYMBOL_STR(gpiod_set_raw_value) },
	{ 0x3bea5325, __VMLINUX_SYMBOL_STR(_dev_info) },
	{ 0xd6b8e852, __VMLINUX_SYMBOL_STR(request_threaded_irq) },
	{ 0xbc477a2, __VMLINUX_SYMBOL_STR(irq_set_irq_type) },
	{ 0xc8db570e, __VMLINUX_SYMBOL_STR(gpiod_to_irq) },
	{ 0xf613a509, __VMLINUX_SYMBOL_STR(gpiod_direction_input) },
	{ 0xab2fec6c, __VMLINUX_SYMBOL_STR(gpiod_direction_output_raw) },
	{ 0x1a2a0b81, __VMLINUX_SYMBOL_STR(gpio_to_desc) },
	{ 0x47229b5c, __VMLINUX_SYMBOL_STR(gpio_request) },
	{ 0x6dfe2a9d, __VMLINUX_SYMBOL_STR(of_get_named_gpio_flags) },
	{ 0x60e51c2f, __VMLINUX_SYMBOL_STR(of_count_phandle_with_args) },
	{ 0xb2d48a2e, __VMLINUX_SYMBOL_STR(queue_work_on) },
	{ 0x27bbf221, __VMLINUX_SYMBOL_STR(disable_irq_nosync) },
	{ 0xfcec0987, __VMLINUX_SYMBOL_STR(enable_irq) },
	{ 0x3c761f83, __VMLINUX_SYMBOL_STR(input_event) },
	{ 0xe707d823, __VMLINUX_SYMBOL_STR(__aeabi_uidiv) },
	{ 0x470b0a4c, __VMLINUX_SYMBOL_STR(i2c_transfer) },
	{ 0xb25266b3, __VMLINUX_SYMBOL_STR(input_unregister_device) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
	{ 0x8c03d20c, __VMLINUX_SYMBOL_STR(destroy_workqueue) },
	{ 0xfe990052, __VMLINUX_SYMBOL_STR(gpio_free) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Cyq,ili2511*");

MODULE_INFO(srcversion, "2799558276E43E21CB43B73");
