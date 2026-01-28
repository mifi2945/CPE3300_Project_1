/**
  ******************************************************************************
  * @file		song.h
  * @brief		Song API Header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef SONG_H_
#define SONG_H_


// all in ms
#define WNOTE	1000*60*4/BPM 		//WHOLE NOTE
#define DHNOTE	1000*60*(2+1)/BPM	//DOTTED HALF NOTE
#define HNOTE	1000*60*2/BPM		//HALF NOTE
#define DQNOTE	1000*60*3/2/BPM		//DOTTED QUARTER NOTE
#define QNOTE	1000*60*1/BPM		//QUARTER NOTE
#define ENOTE	1000*60*1/2/BPM		//EIGHTH NOTE
#define SNOTE	1000*60*1/4/BPM		//SIXTEENTH NOTE

typedef struct {
	int freq;
	int duration;
} note;

void song_init(void);
int get_duration(void);
void play_song(const note*,int);
void stop_song(void);

#endif /* SONG_H_ */
