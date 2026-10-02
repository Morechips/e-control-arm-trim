/* Newlib's __libc_init_array calls _init before the constructor array.
 * The vendor Reset_Handler handles startup; no CRT0 initialization is needed. */
void _init(void)
{
}
