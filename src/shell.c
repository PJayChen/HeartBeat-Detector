/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "stm32f4xx.h"

#define MAX_ARGC 10
#define MAX_CMDNAME 10
#define MAX_CMDHELP 10
#define HISTORY_COUNT 1
#define CMDBUF_SIZE 20

int strcmp(const char *a, const char *b) __attribute__ ((naked));
int strcmp(const char *a, const char *b)
{
        __asm__(
        "strcmp_lop:                \n"
        "   ldrb    r2, [r0],#1     \n"
        "   ldrb    r3, [r1],#1     \n"
        "   cmp     r2, #1          \n"
        "   it      hi              \n"
        "   cmphi   r2, r3          \n"
        "   beq     strcmp_lop      \n"
                "        sub     r0, r2, r3          \n"
        "   bx      lr              \n"
                :::
        );
}



extern xQueueHandle xQueueUARTSend;
extern xQueueHandle xQueueUARTRecvie;


char next_line[3] = {'\n','\r','\0'};
char cmd[HISTORY_COUNT][CMDBUF_SIZE];
int cur_his=0;

/* Command handlers. */
void pwm(int argc, char *argv[]);
void pitch(int argc, char* argv[]);
void roll(int argc, char* argv[]);
void land(int argc, char* argv[]);

/* Enumeration for command types. */
enum {
	CMD_PWM = 0,
	CMD_PITCH,
	CMD_ROLL,
	CMD_LEAD,
	CMD_COUNT
} CMD_TYPE;

/* Structure for command handler. */
typedef struct {
	char cmd[MAX_CMDNAME + 1];
	void (*func)(int, char**);
	char description[MAX_CMDHELP + 1];
} hcmd_entry;

const hcmd_entry cmd_data[CMD_COUNT] = {
	[CMD_PWM] = {.cmd = "pwm", .func = pwm, .description = "pwm"}
	//[CMD_PITCH] = {.cmd = "pitch", .func = pitch, .description = "pitch"},
	//[CMD_ROLL] = {.cmd = "roll", .func = roll, .description = "roll"},
	//[CMD_LEAD] = {.cmd = "land", .func = land, .description = "lead"}
};



void pwm(int argc, char *argv[]){

}

void Delay_5ms( int nCnt_1ms )
{
    int nCnt;
    for(; nCnt_1ms != 0; nCnt_1ms--)
    	for(nCnt = 56580; nCnt != 0; nCnt--);
}


/* ref tim37021 */
int cmdtok(char *argv[], char *cmd)
{
	char tmp[CMDBUF_SIZE];
	int i = 0;
	int j = 0;

	
	while (*cmd != '\0'){
		if(*cmd == ' '){
			cmd++;
		}
		else {
			while (1) {
				/* solve "" & '' in echo command*/
				if ((*cmd == '\'') || (*cmd == '\"')){
					cmd++;
				}
				else if ((*cmd != ' ') && (*cmd != '\0')){
					tmp[i++] = *cmd;
					cmd++;
				}
				else { 
					tmp[i] = '\0';
					i = 0;
					break;
				}		
			}
			strcpy(argv[j++],tmp);
		}
	}
	return j;	
}

void check_keyword()
{
	/*use hardcoded array*/	
	char tok[MAX_ARGC + 1][MAX_CMDHELP];

	char *argv[MAX_ARGC + 1];
	int k = 0;

	for (k;k<MAX_ARGC + 1;k++){
	argv[k] = &tok[k][0];
	}

	int i;
	int argc;

	char cmdstr[CMDBUF_SIZE];
	strcpy(cmdstr, &cmd[cur_his][0]);
	
	argc = cmdtok(argv, cmdstr);

	qprintf(xQueueUARTSend, "command is = %s, argv is = %s\n", argv[0], argv[1]);


	for (i = 0; i < CMD_COUNT; i++) {
		if (!strcmp(cmd_data[i].cmd, argv[0])) {
			cmd_data[i].func(argc, argv);
			break;
		}
	}

	if (i == CMD_COUNT) {

		qprintf(xQueueUARTSend, "no command\n");
 		
	}
}

void shell(void *pvParameters)
{
	char put_ch;
	char *p = NULL;
	char *str ="\rShell:~$";	

	for (;; cur_his = (cur_his + 1) % HISTORY_COUNT) {
		/* need use & that p can work correct, idk why p = cmd[cur_his] can't work */
		p = &cmd[cur_his][0];

		qprintf(xQueueUARTSend, "%s", str);

		while (1) {
			put_ch = receive_byte();			

			if (put_ch == '\r' || put_ch == '\n') {
				*p = '\0';
				qprintf(xQueueUARTSend, "%s",next_line);
				break;
			}
			else if (put_ch== 127 || put_ch == '\b') {
				if (p > &cmd[cur_his][0]) {
					p--;
					qprintf(xQueueUARTSend, "\b \b");
				}
			}
			else if (p - &cmd[cur_his][0] < CMDBUF_SIZE - 1) {
				*(p++) = put_ch;
				qprintf(xQueueUARTSend, "%c",put_ch);
			}

		}
		check_keyword();			
	}
}





