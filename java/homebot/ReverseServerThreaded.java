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
public class ReverseServerThreaded {
	//static BlockingQueue <String> queue = new ArrayBlockingQueue(100);
     
    public static void main(String[] args) { 
		String text="";
		BlockingQueue queue;       
        queue=serverInit();
        
        while (true){
			text=serverLoopOnce(queue);
		}		
    }
    public static BlockingQueue serverInit(){
	  BlockingQueue <String> queue = new ArrayBlockingQueue(100);	
	  try (ServerSocket serverSocket = new ServerSocket(6868)) {     //initiate socket
         System.out.println("Server is listening on port " + "6868");
         Socket socket = serverSocket.accept();
         System.out.println("New client connected");                
         new ServerThread(socket,queue).start();
         String text;
         int counterint=0; 
       } catch (IOException ex) {
            System.out.println("Server exceptionary: " + ex.getMessage());
            ex.printStackTrace();
         }
       return queue;    
	}
	
	public static String serverLoopOnce(BlockingQueue queue){
	  String text="";	
	  try {text=queue.take();}//(10,TimeUnit.MILLISECONDS);}
      catch (InterruptedException ex){
		System.out.println("taking from queue error" + ex.getMessage());
        ex.printStackTrace();
      }
      if (text.length()>0){ 
       System.out.println(text);//msg.getMsg()); 
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
}

