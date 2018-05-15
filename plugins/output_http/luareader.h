
#ifndef __LUAREADER_H_INCLUDED__
#define __LUAREADER_H_INCLUDED__


#ifdef __cplusplus
extern "C" {
#endif

#ifndef LUAREADER_API
#ifdef WIN32
#ifdef LUAREADER_EXPORTS
#define LUAREADER_API __declspec(dllexport)
#else
#define LUAREADER_API __declspec(dllimport)
#endif
#else
#define LUAREADER_API
#endif
#endif

/* 脚本回调应用程序
 context： 同luareader_new返回值
 command：命令
 input(input_len)：输入数据
 output(max_output_size,result)：输出数据
 result：输出数据长度
 example：在脚本内执行	luareader_callback('log', 'test')
*/
typedef int (*luareader_callback_function)(void *context, const char *command, const unsigned char * input, int input_len, unsigned char * output, int max_output_size);

/* 创建对象（加载、执行脚本，设置回调）
 flags：1-script为脚本文件
 script：lua脚本/文件（windows内嵌sysapi对象，linux内嵌usb对象）
 callback：C回调函数
 result：lua上下文
*/
LUAREADER_API void * luareader_new(int flags, const char *script, luareader_callback_function callback);
/* 删除对象
 context：上下文
 result：[0]成功，[-1001]上下文无效
*/
LUAREADER_API int luareader_term(void *context);

/* 获取读卡器列表（内部调用luareader_do_task）
 context：上下文
 reader_names(max_reader_names_size,result)：输出读卡器列表（多字符串格式）
 result：[>0]输出长度，[-1001]上下文无效，[-1,-6]执行脚本错误（必须调用一次luareader_pop_value获取错误信息）
*/
LUAREADER_API int luareader_get_list(void *context, char * reader_names, int max_reader_names_size);

/* 连接读卡器（内部调用luareader_do_task）
 context：上下文
 reader_name：读卡器名称
 result：[0]成功，[-1001]上下文无效，[-1,-6]执行脚本错误（必须调用一次luareader_pop_value获取错误信息）
*/
LUAREADER_API int luareader_connect(void *context, const char * reader_name);

/* 往读卡器执行指令（内部调用luareader_do_task）
 context：上下文
 apdu(apdu_len)：APDU数据
 resp(max_resp_size,result)：指令执行结果
 timeout：超时时间（ms）
 result：[>0]输出长度，[-1001]上下文无效，[-1,-6]执行脚本错误（必须调用一次luareader_pop_value获取错误信息）
*/
LUAREADER_API int luareader_transmit(void *context, const unsigned char * apdu, int apdu_len, unsigned char * resp, int max_resp_size, int timeout);

/* 断开读卡器的连接（内部调用luareader_do_task）
 context：上下文
 result：[0]成功，[-1001]上下文无效，[-1,-6]执行脚本错误（必须调用一次luareader_pop_value获取错误信息）
*/
LUAREADER_API int luareader_disconnect(void *context);

/* 执行指定任务
 context：上下文
 tast_name：任务名称（脚本函数名）
 input(input_len)：输入数据
 output(max_output_size,result)：输出结果
 result：[>=0]输出长度，[-1001]上下文无效，[-1,-6]执行脚本错误（必须调用一次luareader_pop_value获取错误信息）
*/
LUAREADER_API int luareader_do_task(void *context, const char * tast_name, const unsigned char * input, int input_len, unsigned char * output, int max_output_size);

/* 执行脚本
 context：上下文
 str：脚本
 output(max_output_size,result)：输出结果
 result：[>=0]输出长度，[-1001]上下文无效，[-1,-6]执行脚本错误（必须调用一次luareader_pop_value获取错误信息）
*/
LUAREADER_API int luareader_do_string(void *context, const char * str, unsigned char * output, int max_output_size);

/* 弹出栈顶的值
 context：上下文
 value(max_value_size,result)：值
 result：[0]输出长度，[-1001]上下文无效，[-1003]空栈
*/
LUAREADER_API int luareader_pop_value(void *context, char * value, int max_value_size);


#ifdef __cplusplus
}
#endif

#endif
