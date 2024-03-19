/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "std_msgs/String.h"
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

// 包含ROS的头文件
#include "ros/ros.h"
#include "std_msgs/String.h"


int wakeupFlag = 0; // 唤醒标志，用于控制是否开始识别
int resultFlag = 0; //输出结果标志，创建两个全局变量
#define FRAME_LEN	640 // 定义每一帧的长度
#define	BUFFER_SIZE	4096 // 定义缓冲区大小

//ros::Publisher voiceWordsPub; // ROS发布器，用于发布识别结果


// 当接收到唤醒消息时调用的回调函数
void WakeUp(const std_msgs::String::ConstPtr& msg)
{
    wakeupFlag = 1; // Set the wakeup flag when a message is received // 设置唤醒标志位
    ROS_INFO("Wakeup signal received."); // 打印唤醒信号接收信息
}

/* Upload User words */
static int upload_userwords()
{
	char*			userwords	=	NULL;
	size_t			len			=	0;
	size_t			read_len	=	0;
	FILE*			fp			=	NULL;
	int				ret			=	-1;
    // 打开用户词汇文件
	fp = fopen("/home/zhuzhu-jianjian1/Workspace/src/robot_voice/src/userwords.txt", "rb");
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n");
		goto upload_exit;
	}
    // 读取文件长度
	fseek(fp, 0, SEEK_END);
	len = ftell(fp); 
	fseek(fp, 0, SEEK_SET);

    // 分配内存
	userwords = (char*)malloc(len + 1);
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");
		goto upload_exit;
	}

    // 读取文件内容
	read_len = fread((void*)userwords, 1, len, fp); 
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");
		goto upload_exit;
	}
	userwords[len] = '\0';

    // 上传用户词汇
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); //ÉÏ´«ÓÃ»§´Ê±í
    if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
		goto upload_exit;
	}
upload_exit:
	if (NULL != fp)
	{
		fclose(fp);
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords);
		userwords = NULL;
	}
	
	return ret;
}

// 显示识别结果
static void show_result(char *string, char is_over)
{
	printf("\rResult: [ %s ]", string);
	if(is_over)
		putchar('\n');
}

static char *g_result = NULL; // 存储识别结果的全局变量
static unsigned int g_buffersize = BUFFER_SIZE; // 结果缓冲区大小

// 当识别到结果时的回调函数
void on_result(const char *result, char is_last)
{
	if (result) {
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		strncat(g_result, result, size); //g_result 全局变量，里面存储了最终识别到语音的字符串
		show_result(g_result, is_last);
	}
}

// 当开始说话时的回调函数
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);
	}
	g_result = (char*)malloc(BUFFER_SIZE);
	g_buffersize = BUFFER_SIZE;
	memset(g_result, 0, g_buffersize);

	printf("Start Listening...\n");
}

// 当说话结束时的回调函数
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");
	else
		printf("\nRecognizer error %d\n", reason);
}

/* demo send audio data from a file */
// 从文件发送音频数据的演示函数
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0;
	FILE*	f_pcm = NULL;
	char*	p_pcm = NULL;
	unsigned long	pcm_count = 0;
	unsigned long	pcm_size = 0;
	unsigned long	read_size = 0;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	if (NULL == audio_file)
		goto iat_exit;

	f_pcm = fopen(audio_file, "rb");
	if (NULL == f_pcm)
	{
		printf("\nopen [%s] failed! \n", audio_file);
		goto iat_exit;
	}

	fseek(f_pcm, 0, SEEK_END);
	pcm_size = ftell(f_pcm);
	fseek(f_pcm, 0, SEEK_SET);

	p_pcm = (char *)malloc(pcm_size);
	if (NULL == p_pcm)
	{
		printf("\nout of memory! \n");
		goto iat_exit;
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
	if (read_size != pcm_size)
	{
		printf("\nread [%s] error!\n", audio_file);
		goto iat_exit;
	}

	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode);
		goto iat_exit;
	}

	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode);
		goto iat_exit;
	}

	while (1)
	{
		unsigned int len = 10 * FRAME_LEN; /* 200ms audio */
		int ret = 0;

		if (pcm_size < 2 * len)
			len = pcm_size;
		if (len <= 0)
			break;

		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

		if (0 != ret)
		{
			printf("\nwrite audio data failed! error code:%d\n", ret);
			goto iat_exit;
		}

		pcm_count += (long)len;
		pcm_size -= (long)len;		
	}

	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode);
		goto iat_exit;
	}

iat_exit:
	if (NULL != f_pcm)
	{
		fclose(f_pcm);
		f_pcm = NULL;
	}
	if (NULL != p_pcm)
	{
		free(p_pcm);
		p_pcm = NULL;
	}

	sr_stop_listening(&iat);
	sr_uninit(&iat);
}

/* demo recognize the audio from microphone */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("start listen failed %d\n", errcode);
	}
	/* demo 15 seconds recording */
	while(i++ < 5)
		sleep(1);
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("stop listening failed %d\n", errcode);
	}

	sr_uninit(&iat);
}


/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
int main(int argc, char* argv[])
{
 	//***********ROS configuration******************//
 	//Initialize ROS
 	ros::init(argc,argv,"voiceRecognition");//Initialize the name of the node
 	ros::NodeHandle n; //Create the handle of the node
    ros::Publisher voiceWordsPub = n.advertise<std_msgs::String>("voiceWords",1000);
 	ros::Rate loop_rate(1000); //Create the loop, to recognize the voice by loop

 	// Publisher and subscriber
 	// Subscribe the signal of awaking voice recognition
 	ros::Subscriber wakeUpSub = n.subscribe("voiceWakeup",1000,WakeUp);
    std::cout << "wakeupFlag" << wakeupFlag << std::endl;
 	//If there is a waking up topic, then go into the WakeUp function, and execute the recognition function
 	ROS_INFO("Sleeping...");
 	int count = 0;
 	//***********ROS configuration******************//
 	
 	
	int ret = MSP_SUCCESS;
	int upload_on =	1; /* whether upload the user word */
	/* login params, please do keep the appid correct */
	const char* login_params = "appid = 7884b02e, work_dir = .";
	int aud_src = 0; /* from mic or file */

	/*
	* See "iFlytek MSC Reference Manual"
	*/
    const char* session_begin_params =
            "sub = iat, domain = iat, language = en_us, "
            "sample_rate = 16000, "
            "result_type = plain, result_encoding = utf8";

    /* Login first. the 1st arg is username, the 2nd arg is password
     * just set them as NULL. the 3rd arg is login paramertes
     * */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{
		printf("MSPLogin failed , Error code %d.\n",ret);
		goto exit; // login fail, exit the program
	}
////************ Upload user words**************//
//	printf("Want to upload the user words ? \n0: No.\n1: Yes\n");
//	scanf("%d", &upload_on);
//	if (upload_on)
//	{
//		printf("Uploading the user words ...\n");
//		ret = upload_userwords();
//        printf("Test\n");
//		if (MSP_SUCCESS != ret)
//			goto exit;
//		printf("Uploaded successfully\n");
//
//	}
////************ Upload user words**************//

	/*printf("Where the audio comes from?\n"
			"0: From a audio file.\n1: From microphone.\n");
	scanf("%d", &aud_src);
	if(aud_src != 0) {
		#printf("Demo recognizing the speech from microphone\n");
		#printf("Speak in 15 seconds\n");

		demo_mic(session_begin_params);

		printf("15 sec passed\n");
	} else {
		printf("Demo recgonizing the speech from a recorded audio file\n");
		demo_file("wav/iflytek02.wav", session_begin_params); 
	}
	*/
	//*******************循环语音识别和订阅发布话题****************//

    while(ros::ok())
	{
        //printf("ros ok \n");
		if(wakeupFlag)//语音识别唤醒
		{
			ROS_INFO("Wakeup...!!!");
			printf("Demo recognizing the speech from microphone\n");
			printf("Speak in 5 seconds\n");
			
			demo_mic(session_begin_params);
			printf("5 sec passed \n");
			wakeupFlag = 0;
            resultFlag = 1;
		}
		if(resultFlag) //如果识别成功的话需要置位
		{

			resultFlag = 0;
			std_msgs::String msg; //Create Object of message
            //msg.data = "hi"; //testing
			msg.data = g_result; //Copy the final result to the object of message
            std::cout << " g_result: " << msg.data << std::endl;
            voiceWordsPub.publish(msg); //Publish the messgae to the topic of /voiceWords
            printf("Publish done.\n");
		}
		ros::spinOnce();
		//loop_rate.sleep();
		count++;
	}
	//*******************循环语音识别和订阅发布话题****************8//

exit:
	MSPLogout(); // Logout...

	return 0;
}
