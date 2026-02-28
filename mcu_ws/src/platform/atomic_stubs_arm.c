/**
 * @file atomic_stubs_arm.c
 * @brief 64-bit GCC atomic builtins for Cortex-M7 (ARMv7E-M).
 *
 * arm-cortexm7f-eabi has no native 64-bit atomic instructions and the
 * toolchain does not ship libatomic.a, so micro-ROS rclc's calls to
 * __atomic_{load,store,exchange}_8 come up as undefined references.
 *
 * IRQ masking (PRIMASK) gives the required atomicity on a single-core M7.
 */

#ifdef __ARM_ARCH_7EM__

#include <stdint.h>

static inline uint32_t irq_save(void) {
  uint32_t primask;
  __asm__ volatile(
      "mrs %0, primask \n\t"
      "cpsid i"
      : "=r"(primask)::"memory");
  return primask;
}

static inline void irq_restore(uint32_t primask) {
  __asm__ volatile("msr primask, %0" ::"r"(primask) : "memory");
}

__attribute__((used)) uint64_t __atomic_load_8(const volatile void *ptr,
                                               int memorder) {
  (void)memorder;
  uint32_t m = irq_save();
  uint64_t v = *(const volatile uint64_t *)ptr;
  irq_restore(m);
  return v;
}

__attribute__((used)) void __atomic_store_8(volatile void *ptr, uint64_t val,
                                            int memorder) {
  (void)memorder;
  uint32_t m = irq_save();
  *(volatile uint64_t *)ptr = val;
  irq_restore(m);
}

__attribute__((used)) uint64_t __atomic_exchange_8(volatile void *ptr,
                                                   uint64_t desired,
                                                   int memorder) {
  (void)memorder;
  uint32_t m = irq_save();
  uint64_t old = *(volatile uint64_t *)ptr;
  *(volatile uint64_t *)ptr = desired;
  irq_restore(m);
  return old;
}

__attribute__((used)) _Bool __atomic_compare_exchange_8(
    volatile void *ptr, void *expected, uint64_t desired, _Bool weak,
    int success_memorder, int failure_memorder) {
  (void)weak;
  (void)success_memorder;
  (void)failure_memorder;
  uint32_t m = irq_save();
  uint64_t cur = *(volatile uint64_t *)ptr;
  uint64_t exp = *(uint64_t *)expected;
  _Bool ok = (cur == exp);
  if (ok)
    *(volatile uint64_t *)ptr = desired;
  else
    *(uint64_t *)expected = cur;
  irq_restore(m);
  return ok;
}

#endif /* __ARM_ARCH_7EM__ */
