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
 
    public static void main(String[] args) {
        //if (args.length < 1) return;
        //BlockingQueue<Message> queue = new ArrayBlockingQueue<>(100); 
                     //
        //int port = Integer.parseInt(args[0]);
        String GlobalText=""; 
        //Message msg=new Message(" ");
        serverInit();
          
 
        try (ServerSocket serverSocket = new ServerSocket(port)) {     //initiate socket
     
            System.out.println("Server is listening on port " + port);
            while (true) {
                Socket socket = serverSocket.accept();
                System.out.println("New client connected");
                
                //OutputStream output = socket.();    //create socket writing stream
                //PrintWriter writer = new PrintWriter(output, true); //create writing object
 
 
                new ServerThread(socket,queue).start();
                String text;
                int counterint=0;               
                do {					
                    try {GlobalText=queue.take();}//(10,TimeUnit.MILLISECONDS);}
                    catch (InterruptedException ex){
						System.out.println("taking from queue error" + ex.getMessage());
                        ex.printStackTrace();
					}
                    if (GlobalText.length()>0){ 
                      System.out.println(GlobalText);//msg.getMsg());                      
                    }                      
                     //wait(1);                    
                    counterint++;
                } while (counterint<100000000);
                socket.close();                
            }            
 
        } catch (IOException ex) {
            System.out.println("Server exceptionary: " + ex.getMessage());
            ex.printStackTrace();
        }
    }
    public void serverInit(){
	 public BlockingQueue <String> queue = new ArrayBlockingQueue(100); 
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

