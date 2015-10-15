/*
 * DJI_Pro_Link.c
 *
 *  Created on: Mar 12, 2015
 *      Author: wuyuwei
 */

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include "DJI_Pro_Link.h"
#include "DJI_Pro_Hw.h"
#include "DJI_Pro_Codec.h"
#include "DJI_Pro_Rmu.h"

static ACK_Callback_Func Call_APP_Func = 0;
static Req_Callback_Func APP_Recv_Hook = 0;

//send data to serial port, called at last
static void Send_Pro_Data(unsigned char *buf)
{
	ProHeader *pHeader = (ProHeader *)buf;

#ifdef PLATFORM_QT
		//call SerialWrite
    DJI_Pro_Hw::Pro_Hw_Get_Instance()->Pro_Hw_Send(buf,pHeader->length);
#else
		//call SerialWrite
    Pro_Hw_Send(buf,pHeader->length);
#endif
}

void Pro_Link_Recv_Hook(ProHeader *header)
{
	ProHeader *p2header;
	static ACK_Session_Tab * ack_session = Get_ACK_Session_Tab();
	static CMD_Session_Tab * cmd_session = Get_CMD_Session_Tab();
	//TODO: parse the protocol data stream here
	if(header->is_ack == 1)
	{
		if(header->session_id == 1)
		{
			if(cmd_session[1].usage_flag == 1 && cmd_session[1].ack_callback)
			{
				cmd_session[1].ack_callback(header);
				Get_Memory_Lock();
				Free_CMD_Session(&cmd_session[1]);
				Free_Memory_Lock();
			}
		}
		else if(header->session_id > 1 && header->session_id < 32)
		{
			if(cmd_session[header->session_id].usage_flag == 1)
			{
				Get_Memory_Lock();
				p2header = (ProHeader*)cmd_session[header->session_id].mmu->pmem;
				if(p2header->session_id == header->session_id &&
						p2header->sequence_number == header->sequence_number)
				{
                    //printf("%s:Recv Session %d ACK\n",__func__,p2header->session_id);
					Call_APP_Func = cmd_session[header->session_id].ack_callback;
					Free_CMD_Session(&cmd_session[header->session_id]);
					Free_Memory_Lock();
					if(Call_APP_Func)
					{
						Call_APP_Func(header);
					}
				}
				else
				{
					Free_Memory_Lock();
				}
			}
		}
	}
	else
	{
		//TODO,is a request package
		switch(header->session_id)
		{
		case 0:
			Pro_Request_Interface(header);
			break;
		case 1:
		default:
			if(ack_session[header->session_id - 1].session_status == ACK_SESSION_PROCESS)
			{
				printf("%s,This session is waiting for App ack:"
						"session id=%d,seq_num=%d\n",__func__,
						header->session_id,header->sequence_number);
			}
			else if(ack_session[header->session_id - 1].session_status == ACK_SESSION_IDLE)
			{
				if(header->session_id > 1)
				{
					ack_session[header->session_id - 1].session_status = ACK_SESSION_PROCESS;
				}
				Pro_Request_Interface(header);
			}
			else if(ack_session[header->session_id - 1].session_status == ACK_SESSION_USING)
			{
				Get_Memory_Lock();
				p2header = (ProHeader *)ack_session[header->session_id - 1].mmu->pmem;
				if(p2header->sequence_number == header->sequence_number)
				{
					printf("%s:repeat ACK to remote,session id=%d,seq_num=%d\n",
								__func__,header->session_id,header->sequence_number);
					Send_Pro_Data(ack_session[header->session_id - 1].mmu->pmem);
					Free_Memory_Lock();
				}
				else
				{
					printf("%s:same session,but new seq_num pkg,session id=%d,"
							"pre seq_num=%d,""cur seq_num=%d\n",__func__,
							header->session_id,p2header->sequence_number,
							header->sequence_number);
					ack_session[header->session_id - 1].session_status = ACK_SESSION_PROCESS;
					Free_Memory_Lock();
					Pro_Request_Interface(header);
				}
			}
			break;
		}
	}
}

static void Send_Poll(void)
{
	unsigned char i;
	unsigned int cur_timestamp;
	static CMD_Session_Tab * cmd_session = Get_CMD_Session_Tab();
	for(i = 1 ; i < SESSION_TABLE_NUM ; i ++)
	{
		if(cmd_session[i].usage_flag == 1)
		{
			cur_timestamp = Get_TimeStamp();
			if((cur_timestamp - cmd_session[i].pre_timestamp)
					> cmd_session[i].ack_timeout)
			{
				Get_Memory_Lock();
				if(cmd_session[i].retry_send_time > 0 )
				{
					if(cmd_session[i].sent_time >= cmd_session[i].retry_send_time )
					{
						Free_CMD_Session(&cmd_session[i]);
					}
					else
					{
						Send_Pro_Data(cmd_session[i].mmu->pmem);
						cmd_session[i].pre_timestamp = cur_timestamp;
						cmd_session[i].sent_time ++;
					}
				}
				else
				{
					Send_Pro_Data(cmd_session[i].mmu->pmem);
					cmd_session[i].pre_timestamp = cur_timestamp;
				}
				Free_Memory_Lock();
			}

		}

	}
}

static void * PollThread(void * arg)
{
    arg = arg;
	while(1)
	{
		Send_Poll();
		usleep(POLL_TICK * 1000);
	}
	return NULL;
}

static int Start_PollThread(void)
{
	int ret;
	pthread_t A_ARR;
	ret = pthread_create(&A_ARR, 0,PollThread,NULL);
	if(ret != 0)
	{
		return -1;
	}
	return 0;
}

unsigned int Get_TimeStamp(void)
{
	struct timeval cur_time;
	gettimeofday(&cur_time,NULL);
	return (cur_time.tv_sec * 1000) + (cur_time.tv_usec / 1000);
}

void Pro_Link_Setup(void)
{
	DJI_Pro_Rmu_Setup();
	Start_PollThread();
}

void Pro_Config_Comm_Encrypt_Key(const char *key)
{
	sdk_set_encrypt_key_interface(key);
}

static unsigned short Pro_Calc_Length(unsigned short size, unsigned short encrypt_flag)
{
	unsigned short len;
	if(encrypt_flag)
	{
		len = size + sizeof(ProHeader) + 4 +  (16 - size % 16);
	}
	else
	{
		len = size + sizeof(ProHeader) + 4;
	}
	return len;
}

int Pro_Ack_Interface(ProAckParameter *parameter)
{
	unsigned short ret = 0;
	ACK_Session_Tab * ack_session = (ACK_Session_Tab *)NULL;;

	if(parameter->length > PRO_PURE_DATA_MAX_SIZE)
	{
		printf("%s:%d:ERROR,length=%d is oversize\n",__func__,__LINE__,parameter->length);
		return -1;
	}

	if(parameter->session_id == 0)
	{
		;
	}
	else if(parameter->session_id > 0 && parameter->session_id < 32)
	{
		Get_Memory_Lock();
		ack_session = Request_ACK_Session(parameter->session_id,
				Pro_Calc_Length(parameter->length,parameter->need_encrypt));
		if(ack_session == (ACK_Session_Tab*)NULL)
		{
			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);
			Free_Memory_Lock();
			return -1;
		}

		ret = sdk_encrypt_interface(ack_session->mmu->pmem,parameter->buf,
					parameter->length,1,parameter->need_encrypt,
					parameter->session_id,parameter->seq_num);
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);
			Free_Memory_Lock();
			return -1;
		}

		Send_Pro_Data(ack_session->mmu->pmem);
		Free_Memory_Lock();
		ack_session->session_status = ACK_SESSION_USING;
		return 0;
	}

	return -1;
}

//further data processing after `DJI_Pro_App_Send_Data` in `DJI_Pro_APP.cpp`
int Pro_Send_Interface(ProSendParameter *parameter)
{
	unsigned short ret = 0;

	/* CMD_Session_Tab is defined in `DJI_Pro_Rmu.h`
	 * a CMD_Session_Tab is a session object contains every object a protocol frame should have
	 * there are 32 CMD_Session_Tabs in total, 
	 * all are setup in the `Session_Setup` function at init setup i.e. `DJI_Pro_Setup` function
 	 */
	CMD_Session_Tab * cmd_session = (CMD_Session_Tab *) NULL; 

	//Note: global_seq_num is a static value
	static unsigned short global_seq_num = 0;

	//data length evaluation
	if(parameter->length > PRO_PURE_DATA_MAX_SIZE)
	{
		printf("%s:%d:ERROR,length=%d is oversize\n",__func__,__LINE__,parameter->length);
		return -1;
	}

	//different sending procedures based on the session id {0, 1, 2~31}
   switch(parameter->session_mode)
	{
	case 0: //for session 0

		//lock the memory
		Get_Memory_Lock();

		//request the SESSION_0
		//`Request_CMD_Session` is defined in `DJI_Pro_Rmu.cpp`
		cmd_session = Request_CMD_Session(CMD_SESSION_0,Pro_Calc_Length(parameter->length,parameter->need_encrypt));

		//condition: SESSION_0 is not available
		if(cmd_session == (CMD_Session_Tab *)NULL) 
		{
			//unlock memory
			Free_Memory_Lock();

			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);

			//terminate the sending procedure
			return -1;
		}

		//encrypt and build the whole frame, the final result is saved in `cmd_sessino->mmu->pmem`
		//`sdk_encrypt_interface` is defned in `DJI_Pro_Codec`, 
		//`ret` is the return value, which is the total frame length after encryption and CRC calculation
		ret = sdk_encrypt_interface(cmd_session->mmu->pmem,parameter->buf,parameter->length,
				0,parameter->need_encrypt,cmd_session->session_id,global_seq_num);

		//condition: data length == 0 -> encrypt failed
		if(ret == 0) 
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);

			//release the current session state back to available
			Free_CMD_Session(cmd_session);

			//unlock memory
			Free_Memory_Lock();
			
			//terminate the sending procedure
			return -1;
		}

		//send data out (no more processing)
		Send_Pro_Data(cmd_session->mmu->pmem);

		//add the seq index
		global_seq_num ++;

		//release the current session state back to available
		Free_CMD_Session(cmd_session);

		//unlock memory
		Free_Memory_Lock();

		//finish the sending procedure
		break;

	case 1: //for session 1

		//lock the memroy
		Get_Memory_Lock();

		//request SESSION_1
		/*Note:
		* what `Request_CMD_Session` returns is a pointer, instead of a `new` object.
		* therefore, the value inside cmd_session may have already been assigned, even more than once,
		* actually the only thing we care is the seq_id, you will see how we handle it later
		*/
		cmd_session = Request_CMD_Session(CMD_SESSION_1,Pro_Calc_Length(parameter->length,parameter->need_encrypt));

		//condition: SESSION_1 not available
		if(cmd_session == (CMD_Session_Tab *)NULL) 
		{
			//unlock memory
			Free_Memory_Lock();
			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);

			//terminate the sending procedure
			return -1;
		}
		
		//SESSION_1 may be used more than once, global_seq_number should increase if duplicated.
		/* Note:
		*	we only do this for session 1~31, the session 0 doesnt need callback,
		*	therefore the seq id of session 0 doesnt matter
		*/
		if(global_seq_num == cmd_session->pre_seq_num)
		{
			global_seq_num ++;
		}
		
		//encrypt and build the whole frame, the final result is saved in `cmd_sessino->mmu->pmem`
		//`sdk_encrypt_interface` is defned in `DJI_Pro_Codec`, 
		//`ret` is the return value, which is the total frame length after encryption and CRC calculation
		ret = sdk_encrypt_interface(cmd_session->mmu->pmem,parameter->buf,parameter->length,
				0,parameter->need_encrypt,cmd_session->session_id,global_seq_num);

		//condition: data length == 0 -> encrypt failed
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);

			//release the current session state back to available
			Free_CMD_Session(cmd_session);

			//unlock memory
			Free_Memory_Lock();

			//terminate the sending procedure
			return -1;
		}

		//assign the sequence id
		cmd_session->pre_seq_num = global_seq_num ++;

		//assign the callback function
		cmd_session->ack_callback = parameter->ack_callback;

		//assign the timeout
		cmd_session->ack_timeout = (parameter->ack_timeout > POLL_TICK) ?
									parameter->ack_timeout : POLL_TICK;

		//save the current time
		cmd_session->pre_timestamp = Get_TimeStamp();

		//only send once
		cmd_session->sent_time = 1;

		//only retry once for session 1
		cmd_session->retry_send_time = 1;

		//send data out (no more processing)
		Send_Pro_Data(cmd_session->mmu->pmem);

		//unlock memory
		Free_Memory_Lock();

		/*Note:
		*	we do not free the session here
		*	the session state will be set back to available in callback function
		*/
		break;

	case 2:
		//lock memory
		Get_Memory_Lock();

		//request an available session among SESSION_2 ~ SESSION_31
		cmd_session = Request_CMD_Session(CMD_SESSION_AUTO,Pro_Calc_Length(parameter->length,parameter->need_encrypt));

		//condition: no more session available
		if(cmd_session == (CMD_Session_Tab *)NULL)
		{
			//unlock memory
			Free_Memory_Lock();

			printf("%s:%d:ERROR,there is not enough memory\n",__func__,__LINE__);

			//terminate sending procedure
			return -1;
		}

		//current session may be used more than once, global_seq_number should increase if duplicated.
		/* Note:
		*	we only do this for session 1~31, the session 0 doesnt need callback,
		*	therefore the seq id of session 0 doesnt matter
		*/
		if(global_seq_num == cmd_session->pre_seq_num)
		{
			global_seq_num ++;
		}

		
		//encrypt and build the whole frame, the final result is saved in `cmd_sessino->mmu->pmem`
		//`sdk_encrypt_interface` is defned in `DJI_Pro_Codec`, 
		//`ret` is the return value, which is the total frame length after encryption and CRC calculation
		ret = sdk_encrypt_interface(cmd_session->mmu->pmem,parameter->buf,parameter->length,
				0,parameter->need_encrypt,cmd_session->session_id,global_seq_num);


		//condition: data length == 0 -> encrypt failed
		if(ret == 0)
		{
			printf("%s:%d:encrypt ERROR\n",__func__,__LINE__);

			//release the current session state back to available
			Free_CMD_Session(cmd_session);

			//unlock memory
			Free_Memory_Lock();

			//terminate the sending procedure
			return -1;
		}

		//assign the sequence id
		cmd_session->pre_seq_num = global_seq_num ++;

		//assign the callback function
		cmd_session->ack_callback = parameter->ack_callback;

		//assign the timeout
		cmd_session->ack_timeout = (parameter->ack_timeout > POLL_TICK) ?
									parameter->ack_timeout : POLL_TICK;

		//save the current time
		cmd_session->pre_timestamp = Get_TimeStamp();

		//only send once
		cmd_session->sent_time = 1;

		//assign retry time
		cmd_session->retry_send_time = parameter->retry_time;

		//send data out (no more processing)
		Send_Pro_Data(cmd_session->mmu->pmem);

		//unlock memory
		Free_Memory_Lock();

		/*Note:
		*	we do not free the session here
		*	the session state will be set back to available in callback function
		*/
		break;
	}
	return 0;
}

void Pro_App_Recv_Set_Hook(Req_Callback_Func p_hook)
{
    APP_Recv_Hook = p_hook;
}

void Pro_Request_Interface(ProHeader *header)
{
	//TODO call app data handler interface here
    unsigned char buf[2] = {0,0};
    if (APP_Recv_Hook)
	{
        APP_Recv_Hook(header);
	}
    else
    {
        ProAckParameter param;
        printf("%s:Recv request,session id=%d,seq_num=%d\n",
                __func__,header->session_id,header->sequence_number);
        if(header->session_id > 0)
        {
            param.session_id = header->session_id;
            param.seq_num = header->sequence_number;
            param.need_encrypt = header->enc_type;
            param.buf = buf;
            param.length = sizeof(buf);
            Pro_Ack_Interface(&param);
        }
    }
}

void Test_ACK_Callback(ProHeader *header)
{
	printf("%s:session id=%d,sq_num=%d\n",__func__,
			header->session_id,header->sequence_number);
}

void Test_Pro_Link(void)
{
	unsigned char buf[16];
	ProSendParameter param;
const char key[] = {"0000000000000000000000000000000000000000000000000000000000000000"};

	Pro_Config_Comm_Encrypt_Key(key);
#if 0
	//session 0
	buf[0] = 0x11;
	buf[1] = 0x22;
	param.pkg_type = 0;
	param.length = 2;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);

	//session 1
	buf[0] = 0x33;
	buf[1] = 0x44;
	param.pkg_type = 1;
	param.length = 2;
	param.ack_callback = Test_ACK_Callback;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);
#endif
	//session 2~31
	buf[0] = 0x55;
	buf[1] = 0x66;
    param.session_mode = 2;
	param.length = 2;
	param.ack_timeout = 1000;  //unit is ms
	param.ack_callback = Test_ACK_Callback;
	param.retry_time = 1;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);

	//session 2~31
	buf[0] = 0x77;
	buf[1] = 0x88;
    param.session_mode = 2;
	param.length = 2;
	param.ack_timeout = 1000;  //unit is ms
	param.ack_callback = Test_ACK_Callback;
	param.retry_time = 5;
	param.buf = buf;
	param.need_encrypt = 1;
	Pro_Send_Interface(&param);
}
