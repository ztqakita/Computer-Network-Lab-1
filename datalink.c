#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 3000								//Data帧超时时间
#define ACK_TIMER 240								//ACK帧超时时间

#define MAX_SEQ 31										//帧的序号空间，应当为2^n-1
#define NR_BUFS ((MAX_SEQ+1)/2)							//窗口大小
#define inc(k) if(k < MAX_SEQ) k++; else k=0			//计算带模运算的k+1

//帧数据结构
struct FRAME { 
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN]; 
    unsigned int  padding;
};

static unsigned char next_frame_to_send = 0;			//发送窗口的上界
static unsigned char frame_expected = 0;				//发送窗口的下界
static unsigned char ack_expected = 0;					//接收窗口的下界
static unsigned char too_far = NR_BUFS;					//接收窗口的上界

static unsigned char out_buf[NR_BUFS][PKT_LEN];			//接收缓冲区
static unsigned char in_buf[NR_BUFS][PKT_LEN];			//发送缓冲区

static unsigned char nbuffered;							//目前输出的缓存个数，用于判断是否激活网络层

static int phl_ready = 0;								//判断物理层是否准备好
bool arrived[NR_BUFS];									//判断缓存中对应位置是否被占用的缓存区
bool no_nak = true;										//判断之前是否发过nak帧

static bool between(int a, int b, int c){				//如果b在a和c窗口之间，则返回true，否则返回false
	return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

static void put_frame_crc(unsigned char *frame, int len){	//将CRC校验码附在frame上并传送到物理层
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;//数据发送到物理层，物理层状态变为忙碌
}

static void send_data_frame(unsigned char frame_nr){	//将packet打包成frame并发送至物理层
    struct FRAME s;

    s.kind = FRAME_DATA;
    s.seq = frame_nr;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
	memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN);//将缓冲区数据帧存入frame的data域中

    dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);

    put_frame_crc((unsigned char *)&s, 3 + PKT_LEN);		//发送到物理层
	start_timer(frame_nr % NR_BUFS, DATA_TIMER);		//启动数据帧定时器
	stop_ack_timer();									//成功发送后取消ACK定时器
}

static void send_ack_frame(void){						//发送ACK帧
    struct FRAME s;

    s.kind = FRAME_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("Send ACK  %d\n", s.ack);

    put_frame_crc((unsigned char *)&s, 2);
	stop_ack_timer();
}

static void send_nak_frame(void) {						//发送NAK帧
	struct FRAME s;
	
	s.kind = FRAME_NAK;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

	dbg_frame("Send NAK  %d\n", s.ack);
	no_nak = false;

	put_frame_crc((unsigned char*)&s, 2);
	stop_ack_timer();
}

/*void go_back_n(int argc, char** argv)
{
	int event, arg;
	struct FRAME f;
	int len = 0;

	protocol_init(argc, argv);
	lprintf("Designed by Zhang Tianqiu, build: " __DATE__"  "__TIME__"\n");

	disable_network_layer();

	for (;;) {
		event = wait_for_event(&arg);

		switch (event) {
		case NETWORK_LAYER_READY:
			get_packet(buffer);
			nbuffered++;
			send_data_frame();
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;

		case FRAME_RECEIVED:
			len = recv_frame((unsigned char*)& f, sizeof f);
			if (len < 5 || crc32((unsigned char*)& f, len) != 0) {
				dbg_event("**** Receiver Error, Bad CRC Checksum\n");
				break;
			}
			if (f.kind == FRAME_ACK)
				dbg_frame("Recv ACK  %d\n", f.ack);
			if (f.kind == FRAME_DATA) {
				dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
				if (f.seq == frame_expected) {
					put_packet(f.data, len - 7);
					frame_expected = 1 - frame_expected;
				}
				send_ack_frame();
			}
			if (f.ack == frame_nr) {
				stop_timer(frame_nr);
				nbuffered--;
				frame_nr = 1 - frame_nr;
			}
			break;

		case DATA_TIMEOUT:
			dbg_event("---- DATA %d timeout\n", arg);
			send_data_frame();
			break;
		}

		if (nbuffered < 1 && phl_ready)
			enable_network_layer();
		else
			disable_network_layer();
	}
}*/

void selective(int argc, char** argv)
{
	int event;
	int oldest_frame = MAX_SEQ+1;
	int i;
	//int DATA_TIMER = 3000;
	//int bits_received, error_received;
	//bool low_error
	for (i = 0; i < NR_BUFS; i++)	arrived[i] = false;

	nbuffered = 0;
	struct FRAME f;
	int len = 0;										//收到data的长度

	protocol_init(argc, argv);
	lprintf("Designed by Zhang Tianqiu, build: " __DATE__"  "__TIME__"\n");

	disable_network_layer();

	for (;;) {
		event = wait_for_event(&oldest_frame);

		switch (event) {
		case NETWORK_LAYER_READY:
			get_packet(out_buf[next_frame_to_send % NR_BUFS]);	//从网络层获取帧放入输出缓存中
			nbuffered++;										//缓存个数+1
			send_data_frame(next_frame_to_send);
			inc(next_frame_to_send);
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;

		case FRAME_RECEIVED:
			len = recv_frame((unsigned char*)& f, sizeof f);
			//bits_received += len * 8;
			if (len < 5 || crc32((unsigned char*)& f, len) != 0) { //帧损坏
				dbg_event("**** Receiver Error, Bad CRC Checksum\n");
				if (no_nak) {
					send_nak_frame();							//若出现CRC校验错误且未发过NAK帧，则发送NAK帧
					//no_nak = false;
					//stop_ack_timer();
				}
				break;
			}
			if (f.kind == FRAME_ACK)							//若为ACK帧
				dbg_frame("Recv ACK  %d\n", f.ack);

			if (f.kind == FRAME_DATA) {							//若为数据帧
				if ((f.seq != frame_expected) && no_nak) {		//若接受的帧并不是期待的帧且未发过NAK帧
					send_nak_frame();
					no_nak = false;
					stop_ack_timer();
				}
				else
					start_ack_timer(ACK_TIMER);					//启动ACK计时器
				if (between(frame_expected, f.seq, too_far) && (arrived[f.seq % NR_BUFS] == false)) {
					//若该帧落入接受窗口中且其缓存位置未被占用
					dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
					arrived[f.seq % NR_BUFS] = true;			//该位置被占用
					memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);	//帧的data域拷贝至输入缓存
					while (arrived[frame_expected % NR_BUFS]) {	//为保证上交网络层有序性，从frame_expected开始统计
						put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);
						no_nak = true;							
						arrived[frame_expected % NR_BUFS] = false;	//清空缓存，来让下次帧成功存入
						inc(frame_expected);
						inc(too_far);
						start_ack_timer(ACK_TIMER);			
					}
				}
			}

			if ((f.kind == FRAME_NAK) && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) {
				//当帧是NAK，选择重传，重传收到的f.ack的下一帧
				send_data_frame((f.ack + 1) % (MAX_SEQ + 1));
				dbg_frame("Recv NAK  %d\n", f.ack);
			}

			while (between(ack_expected, f.ack, next_frame_to_send)) {
				//最后清空发送窗口中累积确认的ack
				nbuffered--;
				stop_timer(ack_expected % NR_BUFS);
				inc(ack_expected);
			}

			break;

		case DATA_TIMEOUT:
			dbg_event("---- DATA %d timeout\n", oldest_frame);
			if (!between(ack_expected, oldest_frame, next_frame_to_send))
				oldest_frame = oldest_frame + NR_BUFS;
			send_data_frame(oldest_frame);
			break;

		case ACK_TIMEOUT:
			dbg_event("---- DATA %d timeout\n", oldest_frame);
			send_ack_frame();
		}

		if (nbuffered < NR_BUFS && phl_ready)
			enable_network_layer();
		else
			disable_network_layer();
		
		/*if ((error_received << 22) < bits_received) {
			low_error = true;
		}
		else {
			low_error = false;
		}
		if (bits_received > 0x3FFFFFFFFFFF) {
			bits_received >>= 20;
			error_received >>= 20;
		}
		if (low_error) {
			DATA_TIMER = 3000;
		}
		else {
			DATA_TIMER = 5000;
		}*/
	}
}

int main(int argc, char **argv)
{
	selective(argc, argv);
}
