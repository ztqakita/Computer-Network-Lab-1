#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 3000								//Data֡��ʱʱ��
#define ACK_TIMER 240								//ACK֡��ʱʱ��

#define MAX_SEQ 31										//֡����ſռ䣬Ӧ��Ϊ2^n-1
#define NR_BUFS ((MAX_SEQ+1)/2)							//���ڴ�С
#define inc(k) if(k < MAX_SEQ) k++; else k=0			//�����ģ�����k+1

//֡���ݽṹ
struct FRAME { 
    unsigned char kind; /* FRAME_DATA */
    unsigned char ack;
    unsigned char seq;
    unsigned char data[PKT_LEN]; 
    unsigned int  padding;
};

static unsigned char next_frame_to_send = 0;			//���ʹ��ڵ��Ͻ�
static unsigned char frame_expected = 0;				//���ʹ��ڵ��½�
static unsigned char ack_expected = 0;					//���մ��ڵ��½�
static unsigned char too_far = NR_BUFS;					//���մ��ڵ��Ͻ�

static unsigned char out_buf[NR_BUFS][PKT_LEN];			//���ջ�����
static unsigned char in_buf[NR_BUFS][PKT_LEN];			//���ͻ�����

static unsigned char nbuffered;							//Ŀǰ����Ļ�������������ж��Ƿ񼤻������

static int phl_ready = 0;								//�ж�������Ƿ�׼����
bool arrived[NR_BUFS];									//�жϻ����ж�Ӧλ���Ƿ�ռ�õĻ�����
bool no_nak = true;										//�ж�֮ǰ�Ƿ񷢹�nak֡

static bool between(int a, int b, int c){				//���b��a��c����֮�䣬�򷵻�true�����򷵻�false
	return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

static void put_frame_crc(unsigned char *frame, int len){	//��CRCУ���븽��frame�ϲ����͵������
    *(unsigned int *)(frame + len) = crc32(frame, len);
    send_frame(frame, len + 4);
    phl_ready = 0;//���ݷ��͵�����㣬�����״̬��Ϊæµ
}

static void send_data_frame(unsigned char frame_nr){	//��packet�����frame�������������
    struct FRAME s;

    s.kind = FRAME_DATA;
    s.seq = frame_nr;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
	memcpy(s.data, out_buf[frame_nr % NR_BUFS], PKT_LEN);//������������֡����frame��data����

    dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short *)s.data);

    put_frame_crc((unsigned char *)&s, 3 + PKT_LEN);		//���͵������
	start_timer(frame_nr % NR_BUFS, DATA_TIMER);		//��������֡��ʱ��
	stop_ack_timer();									//�ɹ����ͺ�ȡ��ACK��ʱ��
}

static void send_ack_frame(void){						//����ACK֡
    struct FRAME s;

    s.kind = FRAME_ACK;
    s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

    dbg_frame("Send ACK  %d\n", s.ack);

    put_frame_crc((unsigned char *)&s, 2);
	stop_ack_timer();
}

static void send_nak_frame(void) {						//����NAK֡
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
	int len = 0;										//�յ�data�ĳ���

	protocol_init(argc, argv);
	lprintf("Designed by Zhang Tianqiu, build: " __DATE__"  "__TIME__"\n");

	disable_network_layer();

	for (;;) {
		event = wait_for_event(&oldest_frame);

		switch (event) {
		case NETWORK_LAYER_READY:
			get_packet(out_buf[next_frame_to_send % NR_BUFS]);	//��������ȡ֡�������������
			nbuffered++;										//�������+1
			send_data_frame(next_frame_to_send);
			inc(next_frame_to_send);
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;

		case FRAME_RECEIVED:
			len = recv_frame((unsigned char*)& f, sizeof f);
			//bits_received += len * 8;
			if (len < 5 || crc32((unsigned char*)& f, len) != 0) { //֡��
				dbg_event("**** Receiver Error, Bad CRC Checksum\n");
				if (no_nak) {
					send_nak_frame();							//������CRCУ�������δ����NAK֡������NAK֡
					//no_nak = false;
					//stop_ack_timer();
				}
				break;
			}
			if (f.kind == FRAME_ACK)							//��ΪACK֡
				dbg_frame("Recv ACK  %d\n", f.ack);

			if (f.kind == FRAME_DATA) {							//��Ϊ����֡
				if ((f.seq != frame_expected) && no_nak) {		//�����ܵ�֡�������ڴ���֡��δ����NAK֡
					send_nak_frame();
					no_nak = false;
					stop_ack_timer();
				}
				else
					start_ack_timer(ACK_TIMER);					//����ACK��ʱ��
				if (between(frame_expected, f.seq, too_far) && (arrived[f.seq % NR_BUFS] == false)) {
					//����֡������ܴ��������仺��λ��δ��ռ��
					dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
					arrived[f.seq % NR_BUFS] = true;			//��λ�ñ�ռ��
					memcpy(in_buf[f.seq % NR_BUFS], f.data, PKT_LEN);	//֡��data�򿽱������뻺��
					while (arrived[frame_expected % NR_BUFS]) {	//Ϊ��֤�Ͻ�����������ԣ���frame_expected��ʼͳ��
						put_packet(in_buf[frame_expected % NR_BUFS], PKT_LEN);
						no_nak = true;							
						arrived[frame_expected % NR_BUFS] = false;	//��ջ��棬�����´�֡�ɹ�����
						inc(frame_expected);
						inc(too_far);
						start_ack_timer(ACK_TIMER);			
					}
				}
			}

			if ((f.kind == FRAME_NAK) && between(ack_expected, (f.ack + 1) % (MAX_SEQ + 1), next_frame_to_send)) {
				//��֡��NAK��ѡ���ش����ش��յ���f.ack����һ֡
				send_data_frame((f.ack + 1) % (MAX_SEQ + 1));
				dbg_frame("Recv NAK  %d\n", f.ack);
			}

			while (between(ack_expected, f.ack, next_frame_to_send)) {
				//�����շ��ʹ������ۻ�ȷ�ϵ�ack
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
