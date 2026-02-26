/**
 * @file exception_stubs_arm.c
 * @brief Stub implementations for wrapped ARM exception handling functions.
 *
 * These stubs replace the libgcc exception handling routines to eliminate
 * R_ARM_PREL31 relocation errors in large firmware images. The linker's
 * --wrap option redirects calls to these empty stubs.
 *
 * If any code actually tries to use exceptions, it will hang in the
 * infinite loop (or you can add abort() if preferred).
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ARM EABI exception personality routines - used for stack unwinding */
void __wrap___aeabi_unwind_cpp_pr0(void) { while(1); }
void __wrap___aeabi_unwind_cpp_pr1(void) { while(1); }
void __wrap___aeabi_unwind_cpp_pr2(void) { while(1); }

/* GCC personality routine for C++ */
void __wrap___gxx_personality_v0(void) { while(1); }

/* C++ exception handling functions */
void *__wrap___cxa_allocate_exception(unsigned int size) {
    (void)size;
    while(1);
    return (void*)0;
}

void __wrap___cxa_throw(void *thrown_exception, void *tinfo, void (*dest)(void *)) {
    (void)thrown_exception;
    (void)tinfo;
    (void)dest;
    while(1);
}

void __wrap___cxa_rethrow(void) { while(1); }

void *__wrap___cxa_begin_catch(void *exceptionObject) {
    (void)exceptionObject;
    while(1);
    return (void*)0;
}

void __wrap___cxa_end_catch(void) { while(1); }

#ifdef __cplusplus
}
#endif
