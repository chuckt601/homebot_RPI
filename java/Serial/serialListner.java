import java.lang.String;
import java.nio.charset.StandardCharsets;

import com.fazecast.jSerialComm.*;
public class serialListner {

 public static void  main(String[] args) { 

  //SerialPort comPort = SerialPort.getCommPorts()[0];
  SerialPort comPort = SerialPort.getCommPort("dev/ttyACM0");//SerialPort.getCommPorts()[0];

  comPort.openPort();
  try {
  int i=0;
  byte[] writeBuffer= {'w'};

   while (true)
   {
      while (comPort.bytesAvailable() == 0)
         Thread.sleep(20);
     
      byte[] readBuffer = new byte[comPort.bytesAvailable()];
      //byte[] writeBuffer= {'w
 
      int numRead = comPort.readBytes(readBuffer, readBuffer.length);
      //int numRead = comPort.readBytes(readBuffer, readBuffer.length);
      System.out.print(new String(readBuffer, StandardCharsets.UTF_8));
      //System.out.println(" Read " + numRead + " bytes.");
      if (i==25) {
         System.out.println("send w===================================");
         comPort.writeBytes(writeBuffer,1);
          byte temp2=115;
         if (writeBuffer[0]=='w') writeBuffer[0]='s';//temp2;
         else writeBuffer[0]='w';
         i=0;
      } 
      i++;
   }
  } catch (Exception e) { e.printStackTrace(); }
  comPort.closePort();
 }
}

