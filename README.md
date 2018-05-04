This is an experiment with transmitting tones of specific frequencies over sound waves and 
detecting them using a version of DFT (discrete Fourier transform). In this implementation, a
standard numeric keypad like those used in phones is used to enter a number which corresponds
to a specific frequency. This is played on a speaker, and a microphone is used at the other end
to pick up the signal. This is sampled with the Arduino and then the DFT is performed on the
sampled signal to determine the frequency index of maximum correlation and (ideally) correctly
identify the key that was pressed. The whole idea of this project was to mimic DTMF, a dialing 
mechanism for phone lines: https://en.wikipedia.org/wiki/Dual-tone_multi-frequency_signaling. 
I have no idea if any practical applications exist beyond this being an excellent learning tool
for signal analysis.
