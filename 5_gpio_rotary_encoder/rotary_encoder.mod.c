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

static const struct modversion_info ____versions[]
__used
__attribute__((section("__versions"))) = {
	{ 0xfa985410, __VMLINUX_SYMBOL_STR(module_layout) },
	{ 0x51204841, __VMLINUX_SYMBOL_STR(platform_driver_unregister) },
	{ 0xda113ac9, __VMLINUX_SYMBOL_STR(__platform_driver_register) },
	{ 0x1e8b2bb, __VMLINUX_SYMBOL_STR(input_set_abs_params) },
	{ 0xf6ae12d0, __VMLINUX_SYMBOL_STR(of_get_property) },
	{ 0xae372752, __VMLINUX_SYMBOL_STR(of_property_read_u32_array) },
	{ 0x313a8cf4, __VMLINUX_SYMBOL_STR(of_match_device) },
	{ 0x1f592b1a, __VMLINUX_SYMBOL_STR(input_free_device) },
	{ 0x40a6d9af, __VMLINUX_SYMBOL_STR(dev_err) },
	{ 0x243100ec, __VMLINUX_SYMBOL_STR(input_allocate_device) },
	{ 0xcca6874c, __VMLINUX_SYMBOL_STR(kmem_cache_alloc) },
	{ 0x208614a6, __VMLINUX_SYMBOL_STR(kmalloc_caches) },
	{ 0x27e1a049, __VMLINUX_SYMBOL_STR(printk) },
	{ 0x37a0cba, __VMLINUX_SYMBOL_STR(kfree) },
	{ 0xf8a905d3, __VMLINUX_SYMBOL_STR(input_unregister_device) },
	{ 0x16305289, __VMLINUX_SYMBOL_STR(warn_slowpath_null) },
	{ 0xf20dabd8, __VMLINUX_SYMBOL_STR(free_irq) },
};

static const char __module_depends[]
__used
__attribute__((section(".modinfo"))) =
"depends=";

MODULE_ALIAS("of:N*T*Crotary-encoder*");

MODULE_INFO(srcversion, "EB36F10749DB6A7C739D91B");
