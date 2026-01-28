/**
  ******************************************************************************
  * @file		speaker.h
  * @brief		Speaker API header
  *
  * Course:		CPE 2610
  * Section:	131
  * Name:		Mikhail Filippov
  ******************************************************************************
*/

#ifndef SPEAKER_H_
#define SPEAKER_H_


#define B3	247

#define C4	262
#define CS4	277
#define DF4	CS4
#define D4	294
#define DS4	311
#define EF4	DS4
#define E4	330
#define F4	349
#define FS4	370
#define GF4	FS4
#define G4	392
#define GS4	415
#define AF4	GS4
#define A4	440
#define AS4	466
#define BF4	AS4
#define B4	494

#define C5	523
#define CS5	554
#define DF5	CS5
#define D5	587
#define DS5	622
#define EF5	DS5
#define E5	659
#define F5	698
#define FS5	740
#define GF5	FS5
#define G5	784
#define GS5	831
#define AF5	GS5
#define A5	880
#define AS5	932
#define BF5	AS5
#define B5	988

#define C6	1047
#define CS6	1109
#define DF6	CS6
#define D6	1175
#define DS6	1245
#define EF6	DS6
#define E6	1319
#define F6	1397
#define FS6	1480
#define GF6	FS6
#define G6	1568
#define GS6	1661
#define AF6	GS6
#define A6	1760
#define AS6	1865
#define BF6	AS6
#define B6	1976


void speaker_init(void);
void speaker_tone(int);
void speaker_stop(void);
void speaker_volume(unsigned int);

#endif /* SPEAKER_H_ */
