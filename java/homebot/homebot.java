package homebot;
import java.lang.String;
import java.nio.charset.StandardCharsets;
import com.fazecast.jSerialComm.*;
import java.util.Scanner;
import java.io.*;
public class homebot {
	
  public static SerialPort comPort = SerialPort.getCommPort("dev/ttyACM0");
  static StringScan findYaw = new StringScan("yaw");
  static StringScan findmagYaw = new StringScan("magYaw");
  static StringScan findcalibdata = new StringScan("calibration_data");
  //static int matchStringCounter=0;
  public static void main(String[] args) {	  
	  boolean terminate=false;	  
      setup();
      long endTimeMillis=System.currentTimeMillis(); 
      while(!terminate) { 
		long startTimeMillis=System.currentTimeMillis(); 
		//System.out.println(endTimeMillis-startTimeMillis);
		while (startTimeMillis<endTimeMillis+100) startTimeMillis=System.currentTimeMillis();   
		//System.out.println(System.currentTimeMillis());  		
		String inputString=keyboard();		
		if (inputString.length()>0) {          
          switch (inputString.charAt(0)) {
			case 'q':  
              terminate=true;
              readFile("calibration_data.dat");
              shutDown();
			  break;
			case 'w':
			  //serialOut("w");
			  break;			  
	   	  }
	   	  if (terminate) break;	   	  
	   	  serialOut(inputString);
	   	  	   	    
	   	  //fileOut(inputString);	   	            
        }        
	   	String inString=serialIn();
	   	findYaw.scanSerial(inString);
	   	findmagYaw.scanSerial(inString);
	   	findcalibdata.scanSerial(inString);
        endTimeMillis=System.currentTimeMillis();
	  }		  
  } 
  //=======================================================================================   
  public static void setup() {
	  //System.out.println("Hello keyboard 0");         
	   try{	       
           OutputStream fos = new FileOutputStream("test.txt",false);           
           fos.close();           
	       comPort.openPort(); 
	       comPort.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 0, 0);
           }  
         catch (IOException e) {
           System.out.print("Exception");
           }
  }
  //=======================================================================================
  public static void loop() {
  
  
  }
  //=======================================================================================
  public static void shutDown() {
	comPort.closePort();   
    //fos.close();  
  }  
  //=======================================================================================
  public static String keyboard() {
	Scanner scanner = new Scanner(System.in);
    String inputString="";  
	//boolean bytesAvail=false;  
	try{
     int bytesAvail=System.in.available();
	 if (bytesAvail>0)  inputString = scanner.nextLine();
    }
    catch (IOException e) {
      System.out.print("Exception");	
	}
	if (inputString.length()>0){
	  //String excape={char(13)};	
	  inputString=inputString+'$';	
	  //System.out.print("keyboard string=");
	  //System.out.println(inputString);
    } 
	return inputString;
  }
  //=====================================================================================
  public static void fileOut(String outString, String fileName) {
    try{
     OutputStream fos = new FileOutputStream(fileName,true);     
     for(int x = 0; x < outString.length() ; x++) {
      fos.write(outString.charAt(x));   // writes the bytes to file
     }
     fos.close();
     
    }  
    catch (IOException e) {
      System.out.print("file writing Exception");
    }
  }
  //====================================================================================
  public static void serialOut(String outString){	
    byte[] writeBuffer=outString.getBytes(StandardCharsets.UTF_8);       
    comPort.writeBytes(writeBuffer,writeBuffer.length);  
  }
  //====================================================================================
  public static String serialIn(){	  	  
    if (comPort.bytesAvailable() > 0)
	{
      byte[] readBuffer = new byte[comPort.bytesAvailable()]; 
      int numRead = comPort.readBytes(readBuffer, readBuffer.length);
	  String inString=new String(readBuffer, StandardCharsets.UTF_8);
	  String fileName="test.txt";
	  fileOut(inString,fileName); 
	  //System.out.print(readBuffer.length);	  
      System.out.print(inString); 
      return inString; 
    } 
    return "";
  }
  //=====================================================================================
 public static class StringScan {
  
  int matchStringCounter=0;
  String scanOutString="";
  String testString=""; 
  public StringScan(String inString) {      //constructor
    matchStringCounter=0;
    scanOutString="";
    testString=inString;    
  }	  
  public  void scanSerial(String inputString){
	//String testString="yaw= "; 
	//static int matchStringCounter=0;
	String matchString=testString+",";
	for (int i=0;i<inputString.length();i++){
	  if (matchStringCounter>=matchString.length()){
		  if(inputString.charAt(i)=='\n'){
		    System.out.print(scanOutString);
		    scanOutString+=inputString.substring(i,i+1);
		    scanOutString=scanOutString;
		    String fileName=testString+".dat";
		    fileOut(scanOutString,fileName);
		    matchStringCounter=0;
		    scanOutString="";
		  }	
		  else {	  
		    scanOutString+=inputString.substring(i,i+1);
	      }
      }
	  else if(inputString.charAt(i)==matchString.charAt(matchStringCounter)){
	         matchStringCounter++;
	       }
	       else {
	         matchStringCounter=0;
	         scanOutString="";
	       }	   
	}   
   }  
 }
 //========================================================================= 
 static void readFile(String fileName){
    
    try {
      File myObj = new File(fileName);
      Scanner myReader = new Scanner(myObj);
      while (myReader.hasNextLine()) {
        String data = myReader.nextLine();
        System.out.println(data);
        int starting =data.indexOf(",")+1;        
        int ending=data.indexOf(",",starting+1);
        if (ending<0) ending=data.length();
        String subString=data.substring(starting,ending);
        System.out.println(subString);
        float value=Float.parseFloat(subString);
        System.out.println(value);
      }
      myReader.close();
    } catch (FileNotFoundException e) {
      System.out.println("An error occurred.");
      e.printStackTrace();
    }  
  }
//========================================================================

}


