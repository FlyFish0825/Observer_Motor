/**
 * @file sysmem.c
 * @brief newlib堆内存管理的底层实现，只提供_sbrk()。
 * malloc/calloc/realloc最终都会走到这里向上移动堆顶指针。
 * 堆起点由链接脚本符号_end决定，堆顶被限制了预留的MSP栈空间，
 * 因此堆不会长进栈里；但栈溢出仍不会被本文件检测到。
 */
/* Includes */
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

/**
 * 当前堆使用高水位指针，即下一次分配要返回的起始地址。
 * 首次调用_sbrk()时用链接脚本的_end初始化。
 */
static uint8_t *__sbrk_heap_end = NULL;

/**
 * @brief 为newlib堆分配内存，供malloc系列使用。
 *
 * 内存布局（来自链接脚本）：
 * ############################################################################
 * #  .data  #  .bss  #       newlib heap       #          MSP stack          #
 * #         #        #                         # Reserved by _Min_Stack_Size #
 * ############################################################################
 * ^-- RAM start      ^-- _end                             _estack, RAM end --^
 *
 * 分配从_end开始，_Min_Stack_Size决定为MSP栈预留的空间，_estack视为RAM末尾。
 * 若栈实际增长超过预留值，必须调大链接脚本中的_Min_Stack_Size。
 *
 * @param incr 请求增加的字节数。
 * @return 分配到的起始地址；空间不足时置errno=ENOMEM并返回(void*)-1。
 */
void *_sbrk(ptrdiff_t incr)
{
  extern uint8_t _end; /* 链接脚本定义：堆起点 */
  extern uint8_t _estack; /* 链接脚本定义：RAM末尾（栈顶） */
  extern uint32_t _Min_Stack_Size; /* 链接脚本定义：为MSP栈预留的字节数 */
  const uint32_t stack_limit = (uint32_t)&_estack - (uint32_t)&_Min_Stack_Size;
  const uint8_t *max_heap = (uint8_t *)stack_limit;
  uint8_t *prev_heap_end;

  /* 首次调用时把堆顶初始化到_end */
  if (NULL == __sbrk_heap_end)
  {
    __sbrk_heap_end = &_end;
  }

  /* 阻止堆生长进入预留的MSP栈区 */
  if (__sbrk_heap_end + incr > max_heap)
  {
    errno = ENOMEM;
    return (void *)-1;
  }

  /* 返回本次分配的起始地址，再推进堆顶 */
  prev_heap_end = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}

#if defined(__PICOLIBC__)
  // Picolibc expects syscalls without the leading underscore.
  // This creates a strong alias so that
  // calls to `sbrk()` are resolved to our `_sbrk()` implementation.
  // Picolibc期望不带下划线的系统调用名，这里建立强别名，
  // 让sbrk()解析到本文件的_sbrk()实现。
  __strong_reference(_sbrk, sbrk);
#endif
