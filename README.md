# airspy-wsprd -- WSPR daemon for AirSpy receivers

This non-interactive application allows automatic reporting of WSPR spots on WSPRnet. The idea is to allow the use of small computer like RasberryPi or Beaglebone boards, with a simple deamon. This kind of very lightweight setup could run continuously without maintenance and help to increase the WSPR network. The code is massively based on Steven Franke (K9AN) implementation and Joe Taylor (K1JT) work.
 
<h3>Basically, this application :</h3>
- Perform a time alignment (2 mins)
- Start the reception using the AirSpy lib
- Decimate the IQ data (ex. 2.5Msps to 375 sps)
- Decode WSPR signal
- Push the spots on WSPRnet
- Loop...

*Tested with Raspbian GNU Linux, using a RaspberryPi 2*
(18% of one core @1GHz (rx & decimation), and a burst during 10s on the second core)

<h3>Howto :</h3>
1. Install a Linux compatible disto on your device (ex. Raspbian for RaspberryPi)
2. Install dependencies & useful tools (ex. ntp for time synchronization)
   ex: sudo apt-get install build-essential cmake libfftw3-dev libusb-1.0-0-dev curl libcurl4-gnutls-dev ntp 
3. Install airspy library : http://github.com/airspy/host
4. Install airspy-wsprd (this app) : http://github.com/Guenael/airspy-wsrd
   Use a sample "make"
5. Enjoy it with ./airspy_wsprd <your options>

<h3>Tips (for Raspberry Pi):</h3>
- Use ferrite bead to limit the interferences
- Cut off the display (could help to reduce QRN) : /opt/vc/bin/tvservice -o 
- Remove unused modules (ex: /etc/modules: #snd-bcm2835)

<h3>TODO:</h3>
- Port to rtl-sdr
- Port to GnuRadio : gr-wspr
