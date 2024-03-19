#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "qtts.h"
#include "msp_cmn.h"
#include "msp_errors.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
/* wav音频头部格式 */
typedef struct _wave_pcm_hdr
{
	char            riff[4];                // = "RIFF"
	int		size_8;                 // = FileSize - 8
	char            wave[4];                // = "WAVE"
	char            fmt[4];                 // = "fmt "
	int		fmt_size;		// = 下一个结构体的大小 : 16

	short int       format_tag;             // = PCM : 1
	short int       channels;               // = 通道数 : 1
	int		samples_per_sec;        // = 采样率 : 8000 | 6000 | 11025 | 16000
	int		avg_bytes_per_sec;      // = 每秒字节数 : samples_per_sec * bits_per_sample / 8
	short int       block_align;            // = 每采样点字节数 : wBitsPerSample / 8
	short int       bits_per_sample;        // = 量化比特数: 8 | 16

	char            data[4];                // = "data";
	int		data_size;              // = 纯数据长度 : FileSize - 44 
} wave_pcm_hdr;

/* 默认wav音频头部数据 */
wave_pcm_hdr default_wav_hdr = 
{
	{ 'R', 'I', 'F', 'F' },
	0,
	{'W', 'A', 'V', 'E'},
	{'f', 'm', 't', ' '},
	16,
	1,
	1,
	16000,
	32000,
	2,
	16,
	{'d', 'a', 't', 'a'},
	0  
};
/* 文本合成 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
	int          ret          = -1;
	FILE*        fp           = NULL;
	const char*  sessionID    = NULL;
	unsigned int audio_len    = 0;
	wave_pcm_hdr wav_hdr      = default_wav_hdr;
	int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;

	if (NULL == src_text || NULL == des_path)
	{
		printf("params is error!\n");
		return ret;
	}
	fp = fopen(des_path, "wb");
	if (NULL == fp)
	{
		printf("open %s error.\n", des_path);
		return ret;
	}
	/* 开始合成 */
	sessionID = QTTSSessionBegin(params, &ret);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionBegin failed, error code: %d.\n", ret);
		fclose(fp);
		return ret;
	}
	ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSTextPut failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "TextPutError");
		fclose(fp);
		return ret;
	}
	printf("正在合成 ...\n");
	fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
	while (1) 
	{
		/* 获取合成音频 */
		const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);
		if (MSP_SUCCESS != ret)
			break;
		if (NULL != data)
		{
			fwrite(data, audio_len, 1, fp);
		    wav_hdr.data_size += audio_len; //计算data_size大小
		}
		if (MSP_TTS_FLAG_DATA_END == synth_status)
			break;
		printf(">");
		usleep(150*1000); //防止频繁占用CPU
	}
	printf("\n");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSAudioGet failed, error code: %d.\n",ret);
		QTTSSessionEnd(sessionID, "AudioGetError");
		fclose(fp);
		return ret;
	}
	/* 修正wav文件头数据的大小 */
	wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
	
	/* 将修正过的数据写回文件头部,音频文件为wav格式 */
	fseek(fp, 4, 0);
	fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
	fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
	fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
	fclose(fp);
	fp = NULL;
	/* 合成完毕 */
	ret = QTTSSessionEnd(sessionID, "Normal");
	if (MSP_SUCCESS != ret)
	{
		printf("QTTSSessionEnd failed, error code: %d.\n",ret);
	}

	return ret;
}

void toExit()
{
	printf("Press any key to exit ..\n");
	getchar();
	MSPLogout();
}

void voiceWordsCallback(const std_msgs::String::ConstPtr& msg)
{
    std::cout << "voioceWordsCallback internal start......" << std::endl;
    ROS_INFO( "voioceWordsCallback internal start......" );
	char cmd[2000];
	const char* text;
	int ret = MSP_SUCCESS;
	const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
	const char* filename             = "tts_sample1.wav"; //合成的语音文件名称
    std::cout << "Words Voice Callback" << std::endl << std::flush;

    std::cout << "i heard:" << msg->data.c_str() << std::endl;
	text = msg->data.c_str();//合成文本


	std::string dataString = msg->data;
	if(dataString.find("Where")!=std::string::npos || dataString.find("Location")!=std::string::npos)
        {
        char locationString[100] = "I am looking for your location, please wait for a second";
        text = locationString;
        std::cout << text << std::endl;
        }
	else if (dataString.find("Follow me")!=std::string::npos || dataString.find("After me")!=std::string::npos)
    {
        char followString[100] = "Sure, I will follow you when you are shopping";
        struct tm *ptm;
        long ts;

        ts = time(NULL);
        ptm = localtime(&ts);
        std::string string = "Now Time"+std::to_string(ptm->tm_hour)+std::to_string(ptm->tm_min);
        char timeString[40]={0};
        string.copy(timeString,sizeof(string,0));
        text = timeString;
        std::cout << text << std::endl;
        }
	else{
		text = msg->data.c_str();
		}
//
	/* 文本合成 */
	printf("开始合成 ...\n");

	ret = text_to_speech(text, filename, session_begin_params);
	if (MSP_SUCCESS != ret)
	{
		printf("text_to_speech failed, error code: %d.\n", ret);
	}
	printf("合成完毕\n");

	popen("play tts_sample1.wav", "r"); //把合成文本以可读方式播放出来
	sleep(1);
	
}


int main(int argc, char **argv) {
    // 初始化ROS节点，命名为"text_to_speech_node"。
    ros::init(argc, argv, "voice_assistant");

    // 创建节点句柄，用于管理节点资源。
    ros::NodeHandle nh;
    std::cout << "nodehandle" << std::endl;

    // 订阅"voiceWords"主题，队列大小设置为1000。当有消息发布到这个主题时，
    // 调用voiceWordsCallback函数处理接收到的消息。


    // 登录到科大讯飞
    const char* login_params = "appid = 7884b02e, work_dir = .";
    MSPLogin(NULL, NULL, login_params);

    // 使用 while 循环和 ros::ok() 来保持节点运行
    ros::Rate loop_rate(1000); // 控制循环的频率，例如 10 Hz
    std::cout << "login" << std::endl;
    std::cout << "ros::ok" <<ros::ok()<< std::endl;
    while(ros::ok()) {
        //std::cout << "in the loop "<< std::endl;
        ros::Subscriber Wordssub = nh.subscribe("voiceWords", 1000, voiceWordsCallback);
        //std::cout << "voiceWordsCallback" << std::endl;
        ros::spin(); // 处理一次回调队列中的所有回调
        //loop_rate.sleep(); // 根据loop_rate休眠，以保持循环频率
    }

    // 退出前清理资源
    MSPLogout(); // 科大讯飞资源登出
    return 0;
}
//int main(int argc, char **argv) {
//    // 初始化ROS节点，命名为"text_to_speech_node"。
//    ros::init(argc, argv, "text_to_speech_node");
//
//    // 创建节点句柄，用于管理节点资源。
//    ros::NodeHandle nh;
//    std::cout << "nodehandle" << std::endl;
//
//    // 订阅"voiceWords"主题，队列大小设置为1000。当有消息发布到这个主题时，
//    // 调用voiceWordsCallback函数处理接收到的消息。
//    ros::Subscriber sub = nh.subscribe<std_msgs::String>("voiceWords",1000, voiceWordsCallback);
//    //ros::Subscriber sub = nh.subscribe("voiceWords", 1000, voiceWordsCallback);
//    std::cout << "voiceWordsCallback" << std::endl;
//    // 登录到科大讯飞
//    const char* login_params = "appid = your_appid, work_dir = .";
//    MSPLogin(NULL, NULL, login_params);
//
//    // ROS消息回环，直到接收到ROS终止信号
//    ros::spin();
//
//    // 退出前清理资源
//    MSPLogout(); // 科大讯飞资源登出
//    return 0;
//}

