/**
 * drk_network.c
 */

#include "drk/drk_network.h"

#include <stdio.h>

int drk_update_rssi(struct drk_rssi * ptr)
{  
  if (ptr == NULL)
    return -1;
  FILE *fp;
  char result[1024];

  /* Open the command for reading. */
  fp = popen("iwconfig ath0 | grep 'Link'", "r");
  if (fp == NULL)
  {
    perror("Failed to run command\n");
    exit(EXIT_FAILURE);
  }

  char s[500];
  /* Read the output a line at a time - output it. */
  while (fgets(result, sizeof(result) - 1, fp) != NULL)
  {
    sprintf(s,"%s%s",s, result);
  }
  printf("%s",s);
  pclose(fp);
  
  /* Ex: Link Quality:38/94  Signal level:-57 dBm  Noise level:-96 dBm */
  int matches, qual_num = 1, qual_den = 1, signal = 1, noise = 1;
  matches = sscanf(s,
    "Link Quality:%d/%d  Signal level:%d dBm  Noise level:%d dBm",
    &qual_num, &qual_den, &signal, &noise);
  if (matches != 4) return -1;
  double quality = qual_num / qual_den;
  ptr->quality = quality;
  ptr->signal = signal;
  ptr->noise = noise;
  return 1;
}

// to be deleted
