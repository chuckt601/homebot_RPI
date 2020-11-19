import java.lang.String;
import java.nio.charset.StandardCharsets;
import com.fazecast.jSerialComm.*;
import java.util.Scanner;
import java.io.*;
public class terminal {
	
  private static SerialPort comPort = SerialPort.getCommPort("dev/ttyACM0");
  
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
	   	serialIn();
        endTimeMillis=System.currentTimeMillis();
	  }		  
  } 
  //=======================================================================================   
  public static void setup() {
	  System.out.println("Hello keyboard 0");         
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
	return inputString;
  }
  //=====================================================================================
  public static void fileOut(String outString) {
    try{
     OutputStream fos = new FileOutputStream("test.txt",true);     
     for(int x = 0; x < outString.length() ; x++) {
      fos.write(outString.charAt(x));   // writes the bytes to file
     }
     fos.close();
    }  
    catch (IOException e) {
      System.out.print("Exception");
    }
  }
  //====================================================================================
  public static void serialOut(String outString){	
    byte[] writeBuffer=outString.getBytes(StandardCharsets.UTF_8);    
    comPort.writeBytes(writeBuffer,1);  
  }
  //====================================================================================
  public static void serialIn(){	  
	if (comPort.bytesAvailable() > 0) {
      byte[] readBuffer = new byte[comPort.bytesAvailable()];      
      int numRead = comPort.readBytes(readBuffer, readBuffer.length);      
	  String inString=new String(readBuffer, StandardCharsets.UTF_8);
	  fileOut(inString); 	  
      System.out.print(inString);  
    }    
  }
  //=====================================================================================
}
