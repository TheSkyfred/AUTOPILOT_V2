#include "Arduino.h"

#include "buzzer.h"
#include "def.h"
#include "config.h"

const int buzzerChannel = 0; // Canal PWM pour le buzzer


int start_melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

int start_melody_noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};


void init_buzzer() {
  // Configurer le canal PWM pour le buzzer
  ledcSetup(buzzerChannel, 2000, 8); // 2000 Hz par défaut, résolution de 8 bits
  ledcAttachPin(BUZZERPIN, buzzerChannel);

  // Parcourir les notes de la mélodie
  for (int thisNote = 0; thisNote < 8; thisNote++) {
    int noteDuration = 1000 / start_melody_noteDurations[thisNote];
    if (start_melody[thisNote] != 0) {  // Si la note n'est pas un silence
      ledcWriteTone(buzzerChannel, start_melody[thisNote]);
    } else {
      ledcWriteTone(buzzerChannel, 0);  // Silence
    }

    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);

    // Arrêter la tonalité entre les notes
    ledcWriteTone(buzzerChannel, 0);
  }
}

void play_buzzer(int note, int duration) {

    // Jouer la note
    ledcWriteTone(buzzerChannel, note);
    delay(duration);

    // Arrêter le son
    ledcWriteTone(buzzerChannel, 0);
}


void takeoff_buzzer(float buzzer_frequency){
    ledcWriteTone(buzzerChannel, buzzer_frequency); // Jouer la fréquence sur le buzzer
}

void buzzer_mute(){

          ledcWriteTone(buzzerChannel, 0); // Pas de son si en dehors de la plage

};

