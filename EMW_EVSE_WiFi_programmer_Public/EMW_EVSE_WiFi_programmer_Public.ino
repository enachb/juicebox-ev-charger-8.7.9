/*
this is WiFI config sketch for EMW JuiceBox

The first version of this did not do anything - the only purpose of it was to disable all serial comms from Arduino
so there is no conflict with the commands from the PC
*/

#include <SoftwareSerial.h>
#include <WiFlyHQ.h>

const char mySSID[] = "YOUR SSID"; // not critical - could be set via WPS
const char myPWD[] = "YOUR PWD"; // not critical - could be set via WPS


SoftwareSerial wifiSerial(2,4);
WiFly wifly;

const char myHOST[] = "50.21.181.240"; // main EMW server
const int myPORT=8042; // matches UDP listener on the server
const int cmddelay=300; // 300ms delay between commands


void terminal();



void setup() {
    char buf[32];

    Serial.begin(9600);
    Serial.println("Starting");
    Serial.print("Free memory: ");
    Serial.println(wifly.getFreeMemory(),DEC);

    wifiSerial.begin(9600);
    if (!wifly.begin(&wifiSerial, NULL)) { // by default. no error output into &Serial
        Serial.println("Failed to start wifly");
	terminal();
    } else {
      Serial.println("Success starting wifly");
    }
    
    // now init
    wifly.factoryRestore();
  
    boolean success=1;

    if(wifly.sendCommand(PSTR("set ip dhcp"), 1)) {
      Serial.println("DHCP NOW ON"); 
    } else success=0;
    delay(cmddelay);
    
    if(wifly.sendCommand(PSTR("set wlan auth"), 4)) {
      Serial.println("WLAN AUTH NOW 4 (WPA2)"); // pp31 - 0=open, 4=WPA2
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set wlan join"), 1)) {
      Serial.println("WLAN JOIN NOW 1"); // join the wifi network with stored params, SSID, pass, etc.
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set wlan ssid"), mySSID, 0)) { // pp35
      Serial.print("SSID NOW ");
      Serial.println(mySSID); 
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set wlan phrase"), myPWD, 0)) {
      Serial.print("PASSPHRASE NOW ");
      Serial.println(myPWD); 
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set ip protocol"), 1)) {
      Serial.println("IP PROTOCOL NOW 1"); // UDP=1, pp18
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set ip remote"), myPORT)) {
      Serial.println("IP REMOTE PORT NOW set"); // 
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set ip host"), myHOST, 0)) {
      Serial.print("IP HOST NOW "); 
      Serial.println(myHOST);  
    } else success=0;
    delay(cmddelay);

    
    // set communication options 
    if(wifly.sendCommand(PSTR("set comm time"), 0)) {
      Serial.println("COMM TIME NOW 0"); // no timeout for packet
    } else success=0;
    delay(cmddelay);

    if(wifly.sendCommand(PSTR("set comm match"), 0xD)) {
      Serial.println("COMM MATCH NOW 0xD"); // send on carriage return 
    } else success=0;
    delay(cmddelay);


    // set the button to run WPS app
    if(wifly.sendCommand(PSTR("set sys launch_string"), "wps_app", 0)) {
      Serial.println("WPS IS NOW ON"); // enable WPS app on button press (pp22) 
    } else success=0;
    delay(cmddelay);

    
    // set the module to pull real time from the tine server (default is 64.90.182.55:123 - Pacific time zone)
    if(wifly.sendCommand(PSTR("set time enable"), 10)) { // every 10 minutes
      Serial.println("RTC IS NOW ON from 64.90.182.55:123"); // pp27,68 
    } else success=0;
    delay(cmddelay);

    
    //================ prepare the WiFly module for accepting commands from JuiceBox =================
    // THIS IS NOT YET FUNCTIONAL - DO NOT RELY!
    // disable LED use by the module's firmware (to enable their use by JuiceBox to indicate status etc)
    if(wifly.sendCommand(PSTR("set sys iofunc"), 0x7)) { // pp27
      Serial.println("LEDs are now available for JuiceBox's use"); 
    } else success=0;
    delay(cmddelay);

    // there will be two main ways to send commands to the JuiceBox via network: 
    //     1. Local commands from the JuiceBox smartphone app (to be developed) - the smartphone is 
    //        connected to the same local wifi router as JuiceBox. Latency will be ~zero for both 
    //        monitoring stats and commands
    //     2. Remote commands from the JuiceBox website or smartphone app when the smartphone is NOT
    //        connected to the same local network. In these cases, to avoid tricky port forwarding setup
    //        on the local routers, JuiceBox will use HTTP pulls from EMW server to get commands
    //        This means that there will be high latency in executing commands (up to 10 minutes - to
    //        avoid overloading JuiceBox servers with too many requests for comments). Therefore, 
    //        you should think about remote commands as batch commands to be executed 'soon', not 'right now'
    //        You will also experience similar latency in display of the JuiceBox status / charging stats
    //        when connected remotely
    // IN THE INITIAL VERSIONS OF JUICEBOX, ONLY REMOTE MONITORING WILL BE IMPLEMENTED - this means
    // NO commands can be sent via network to JuiceBox and NO local smartphone monitoring is possible
    // We expect these features to be implemented in V9.0+ of the JuiceBox  
    //================ END preparing WiFly for commands from JuiceBox  ===============================
   
    // activate the settings now
    wifly.save();
    wifly.reboot();
    
    if(success) {
      Serial.println("WiFly initialization COMPLETE!");
    } else Serial.println("WiFi FAILED to initialize completely! Close & restart Serial Monitor to re-launch the initialization script. If failed 5 times in a row, contact EMW.");
    
    while(!wifly.isAssociated()) {
      Serial.print(".");
      delay(100);
    }
    Serial.println("Associated!");
   
    // terminal();
    
}


int i=0;

void loop() {
  // put your main code here, to run repeatedly: 

   // send some test data
   // this emulates the 
   wifiSerial.print("test data: ");
   wifiSerial.println(i++);
   
   delay(2000);
  
}


void terminal()
{
    Serial.println("Terminal ready");
    while (1) {
	if (wifly.available() > 0) {
	    Serial.write(wifly.read());
	}

	if (Serial.available()) {
	    wifly.write(Serial.read());
	}
    }
}

