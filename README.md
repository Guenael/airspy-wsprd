 #airspy-wsprd -- WSPR Daemon for AirSpy receivers

 This non-interactive application allow to report WSPR spots on WSPRnet. The idea is to allows the use on this daemon on small computer like RasberryPi or Beaglebone boards. This kind of very lightweight setup could run continuously without maintenance and help to increase the WSPR network. The code is massively based on Steven Franke (K9AN) implementation and Joe Taylor (K1JT) work.
 
<h3>Basically, this application :</h3>
- Perform a time alignment (2 mins)
- Start the reception using the AirSpy lib
- Decimate the IQ data (2.5Msps to 375 sps)
- Decode WSPR signal
- Push the data on WSPRnet
- Loop...

*Tested with Raspbian GNU Linux, using a RaspberryPi 2*
(67% of one core @1GHz)

<h3>Howto :</h3>
1. Install a Linux compatible disto on your device (ex. Raspbian for RaspberryPi)
2. Install dependencies & useful tools (ex. ntp for time synchronization)
   sudo apt-get install build-essential cmake libfftw3-dev libusb-1.0-0-dev curl libcurl4-gnutls-dev ntp 
3. Install airspy library : http://github.com/airspy/host
4. Install airspy-wsprd (this app) : http://github.com/Guenael/airspy-wsrd
   make
5. Test it with ./airspy_wsprd <your options>

<h3>Tips (for RasberrpyPi):</h3>
- Use ferrite bead to limit the interference
- Cut off the display : /opt/vc/bin/tvservice -o 
- Remove unused modules : /etc/modules: #snd-bcm2835

<h3>TODO:</h3>
- fix date & add struct
- Use pthreads to dispatch the load during the decoding/receiving
- Doc & cleanup
- Port to rtl-sdr
- Port to GnuRadio : gr-wspr
