import java.io.*;
import java.net.*;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
 
/**
 * This program demonstrates a simple TCP/IP socket server that echoes every
 * message from the client in reversed form.
 * This server is single-threaded.
 *
 * @author www.codejava.net
 */
public class ServerThreaded {
	static BlockingQueue <String> queue = new ArrayBlockingQueue(100);
     
    
    public ServerThreaded(){	  	
	  try (ServerSocket serverSocket = new ServerSocket(6868)) {     //initiate socket
         System.out.println("Server is listening on ports " + "6868");
         Socket socket = serverSocket.accept();
         System.out.println("New connected");                
         new ServerThread(socket,queue).start();         
       } catch (IOException ex) {
            System.out.println("Server exceptionary: " + ex.getMessage());
            ex.printStackTrace();
       }           
	}
	//======================================================================================================
	
	public static String serverLoopOnce(){
	  String text="";	
	  try {
		  text=queue.poll(1L,TimeUnit.MICROSECONDS);
		  queue.clear();
	  }
      catch (InterruptedException ex){
		System.out.println("taking from queue error" + ex.getMessage());
        ex.printStackTrace();
      }
      if (text!=null){ 
	   //System.out.println("null text in queue"); 	  
      //System.out.println(text);//msg.getMsg()); 
	  }
	  return text;
    }
    public static void wait(int ms) {
      try {
        Thread.sleep(ms);
      }
      catch(InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }
    //==========================================================================
    public static void main(String[] args) { 
		String text="";
		//BlockingQueue queue;       
        //ServerThreadeded();
        ServerThreaded a = new ServerThreaded();
        while (true){
			text=a.serverLoopOnce();
		}		
    }
}

