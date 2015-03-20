/**
 * MPC Algorithm Testing and Implementing code
 * communication format file: test.h
 *                                      by Weichen Li
 *                                      2012.8.18
 * @file    test.h
 * @brief   Support file of MPC Algorithm Testing and Implementing code.
 * @details This file is the header of test.c.  
 */

#ifndef _TEST_H_
#define _TEST_H_

#ifdef __cplusplus
extern "C" {
#endif
  msg_t TestThread(void *p);
  void output_number(void *p, msg_t Time);
  void usr_fprint(float *number);
  void usr_intprint(int32_t *number);
  void test_printn(uint32_t n);
  void test_print(const char *msgp);
  void test_println(const char *msgp);
#if CH_DBG_THREADS_PROFILING
  void test_cpu_pulse(unsigned duration);
#endif
#if defined(WIN32)
  void ChkIntSources(void);
#endif
#ifdef __cplusplus
}
#endif

#endif /* _TEST_H_ */

