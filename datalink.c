#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "protocol.h"
#include "datalink.h"

#define DATA_TIMER 3000								//Data֡��ʱʱ��
#define ACK_TIMER 280								//ACK֡��ʱʱ��

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

static bool between(int a, int b, int c) {				//���b��a��c����֮�䣬�򷵻�true�����򷵻�false
	return ((a <= b) && (b < c)) || ((c < a) && (a <= b)) || ((b < c) && (c < a));
}

static void put_frame_crc(unsigned char* frame, int len) {	//��CRCУ���븽��frame�ϲ����͵������
	*(unsigned int*)(frame + len) = crc32(frame, len);
	send_frame(frame, len + 4);
}

static void send_data_frame(unsigned char frame_nr, unsigned char frame_expected, unsigned char *packet, size_t frame_len) {	//��packet�����frame�������������
	struct FRAME s;

	s.kind = FRAME_DATA;
	s.seq = frame_nr;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);
	memcpy(s.data, packet, frame_len);//������������֡����frame��data����

	dbg_frame("Send DATA %d %d, ID %d\n", s.seq, s.ack, *(short*)s.data);

	put_frame_crc((unsigned char*)& s, 3 + frame_len);		//���͵������
	//start_timer(frame_nr % NR_BUFS, DATA_TIMER);		//��������֡��ʱ��

	//stop_ack_timer();									//�ɹ����ͺ�ȡ��ACK��ʱ��
}

static void send_ack_frame(unsigned char frame_expected) {						//����ACK֡
	struct FRAME s;

	s.kind = FRAME_ACK;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

	dbg_frame("Send ACK  %d\n", s.ack);

	put_frame_crc((unsigned char*)& s, 2);
	//stop_ack_timer();
}

static void send_nak_frame(unsigned char frame_expected) {						//����NAK֡
	struct FRAME s;

	s.kind = FRAME_NAK;
	s.ack = (frame_expected + MAX_SEQ) % (MAX_SEQ + 1);

	dbg_frame("Send NAK  %d\n", s.ack);

	put_frame_crc((unsigned char*)& s, 2);
	//stop_ack_timer();
}

void go_back_n(int argc, char** argv)											//δʹ��NAK֡
{
	int event, arg, i;
	struct FRAME f;
	
	unsigned char nbuffered = 0;												//Ŀǰ����Ļ�������������ж��Ƿ񼤻������
	unsigned char buffer[MAX_SEQ + 1][PKT_LEN];									//���������
	unsigned char next_frame_to_send = 0;										//���ʹ��ڵ��Ͻ�
	unsigned char frame_expected = 0;											//���ʹ��ڵ��½�
	unsigned char ack_expected = 0;												//���մ�������λ��

	int packet_len[MAX_SEQ + 1];												//�������ݰ��Ĵ�С
	int frame_len = 0;															//֡��

	bool phl_ready = false;

	protocol_init(argc, argv);
	lprintf("Designed by Zhang Tianqiu, build: " __DATE__"  "__TIME__"\n");

	disable_network_layer();

	for (;;) {
		event = wait_for_event(&arg);

		switch (event) {
		case NETWORK_LAYER_READY:
			packet_len[next_frame_to_send] = get_packet(buffer[next_frame_to_send]);
			nbuffered++;
			send_data_frame(next_frame_to_send, frame_expected, buffer[next_frame_to_send], packet_len[next_frame_to_send]);
			start_timer(next_frame_to_send, DATA_TIMER);
			phl_ready = false;
			inc(next_frame_to_send);
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = true;
			break;

		case FRAME_RECEIVED:
			frame_len = recv_frame((unsigned char*)& f, sizeof f);
			if (frame_len < 5 || crc32((unsigned char*)& f, frame_len) != 0) {
				dbg_event("**** Receiver Error, Bad CRC Checksum\n");
				break;
			}

			if (f.kind == FRAME_DATA) {
				dbg_frame("Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
				if (f.seq == frame_expected) {
					put_packet(f.data, frame_len - 7);
					inc(frame_expected);
				}
			}

			while (between(ack_expected, f.ack, next_frame_to_send)) {
				nbuffered--;
				stop_timer(ack_expected);
				inc(ack_expected);
			}
			
			break;

		case DATA_TIMEOUT:
			dbg_event("---- DATA %d timeout\n", arg);
			next_frame_to_send = ack_expected;
			for (i = 1; i <= nbuffered; i++) {
				send_data_frame(next_frame_to_send, frame_expected, buffer[next_frame_to_send], packet_len[next_frame_to_send]);
				start_timer(next_frame_to_send, DATA_TIMER);
				inc(next_frame_to_send);
			}
			phl_ready = false;
			break;
		}

		if (nbuffered < 1 && phl_ready)
			enable_network_layer();
		else
			disable_network_layer();
	}
}

void selective(int argc, char** argv)
{
	unsigned char next_frame_to_send = 0;			//���ʹ��ڵ��Ͻ�
	unsigned char frame_expected = 0;				//���ʹ��ڵ��½�
	unsigned char ack_expected = 0;					//���մ��ڵ��½�
	unsigned char too_far = NR_BUFS;				//���մ��ڵ��Ͻ�

	unsigned char out_buf[NR_BUFS][PKT_LEN];		//���ջ�����
	unsigned char in_buf[NR_BUFS][PKT_LEN];			//���ͻ�����

	unsigned char nbuffered;						//Ŀǰ����Ļ�������������ж��Ƿ񼤻������

	int phl_ready = 0;								//�ж�������Ƿ�׼����
	bool arrived[NR_BUFS];								//�жϻ����ж�Ӧλ���Ƿ�ռ�õĻ�����
	bool no_nak = true;									//�ж�֮ǰ�Ƿ񷢹�nak֡
	int packet_len[MAX_SEQ + 1];						//�������ݰ��Ĵ�С
	int frame_len = 0;									//֡��

	int event;
	int oldest_frame = MAX_SEQ + 1;
	int i;
	//int bits_received, error_received;
	//bool low_error = false;
	for (i = 0; i < NR_BUFS; i++)	arrived[i] = false;

	nbuffered = 0;
	struct FRAME f;

	protocol_init(argc, argv);
	lprintf("Designed by Zhang Tianqiu, build: " __DATE__"  "__TIME__"\n");

	disable_network_layer();

	for (;;) {
		event = wait_for_event(&oldest_frame);

		switch (event) {
		case NETWORK_LAYER_READY:
			get_packet(out_buf[next_frame_to_send % NR_BUFS]);	//��������ȡ֡�������������
			nbuffered++;										//�������+1
			send_data_frame(next_frame_to_send, frame_expected, out_buf[next_frame_to_send % NR_BUFS], PKT_LEN);
			start_timer(next_frame_to_send % NR_BUFS, DATA_TIMER);
			inc(next_frame_to_send);
			break;

		case PHYSICAL_LAYER_READY:
			phl_ready = 1;
			break;

		case FRAME_RECEIVED:
			frame_len = recv_frame((unsigned char*)& f, sizeof f);
			//bits_received += len * 8;
			if (frame_len < 5 || crc32((unsigned char*)& f, frame_len) != 0) { //֡��
				dbg_event("**** Receiver Error, Bad CRC Checksum, ID: %d\n", * (short*)f.data);
				if (no_nak) {
					send_nak_frame(frame_expected);							//������CRCУ�������δ����NAK֡������NAK֡
					no_nak = false;
					stop_ack_timer();
				}
				break;
			}
			if (f.kind == FRAME_ACK)							//��ΪACK֡
				dbg_frame("**** Recv ACK  %d\n", f.ack);

			if (f.kind == FRAME_DATA) {							//��Ϊ����֡
				if ((f.seq != frame_expected) && no_nak) {		//�����ܵ�֡�������ڴ���֡��δ����NAK֡
					send_nak_frame(frame_expected);
					no_nak = false;
					stop_ack_timer();
				}
				else
					start_ack_timer(ACK_TIMER);					//����ACK��ʱ��
				if (between(frame_expected, f.seq, too_far) && (arrived[f.seq % NR_BUFS] == false)) {
					//����֡������ܴ��������仺��λ��δ��ռ��
					dbg_frame("**** Recv DATA %d %d, ID %d\n", f.seq, f.ack, *(short*)f.data);
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
				dbg_frame("**** Recv NAK  %d\n", f.ack);
				send_data_frame((f.ack + 1) % (MAX_SEQ + 1), frame_expected, out_buf[(f.ack + 1) % NR_BUFS], PKT_LEN);
				start_timer((f.ack + 1) % NR_BUFS, DATA_TIMER);
				stop_ack_timer();
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
			send_data_frame(oldest_frame, frame_expected, out_buf[oldest_frame % NR_BUFS], PKT_LEN);
			start_timer(oldest_frame% NR_BUFS, DATA_TIMER);
			stop_ack_timer();
			break;

		case ACK_TIMEOUT:
			dbg_event("---- DATA %d ACK timeout\n", oldest_frame);
			send_ack_frame(frame_expected);
			stop_ack_timer();
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



int main(int argc, char** argv)
{
	if (argc < 2)
	{
		lprintf("Too less arguments.\n");
		return 0;
	}
	switch (argv[1][strlen(argv[1]) - 1])
	{
	case 'G':
		argv[1][strlen(argv[1]) - 1] = 0;
		go_back_n(argc, argv);
		break;

	case 'S':
		argv[1][strlen(argv[1]) - 1] = 0;
		selective(argc, argv);
		break;

	default:
		lprintf("Please select a mode.\n");
		return 0;
	}
}