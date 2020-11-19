//import java.io.InputStream;
import java.util.Scanner;
import java.io.*;
public class keyboard {
	
  public static void main(String[] args) {
	  Scanner scanner = new Scanner(System.in);
      String inputString=""; 
	  boolean terminate=false;
	  boolean bytesAvail=false;
	  
      setup(); 
      while(!terminate) {
  		bytesAvail=scanner.hasNext();//System.in.available();
		if (bytesAvail)  inputString = scanner.nextLine(); 
		else inputString="";  
        System.out.println(inputString);
        if (inputString.charAt(0)=='q') {
			terminate=true;
			break;
		}
        try{
           OutputStream fos = new FileOutputStream("test.txt",true);
           for(int x = 0; x < inputString.length() ; x++) {
             fos.write(inputString.charAt(x));   // writes the bytes to file
             }
           fos.close();
           }  
         catch (IOException e) {
           System.out.print("Exception");
           }
        }   	 
		  
  }    
  public static void setup() {
	  System.out.println("Hello keyboard 0");
         
	   try{	       
           OutputStream fos = new FileOutputStream("test.txt",false);           
           fos.close();
            
           }  
         catch (IOException e) {
           System.out.print("Exception");
           }
  }
  public static void loop() {
  
  
  }
  public static void Shutdown() {
  
  
  }  
}
