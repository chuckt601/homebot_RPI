import java.io.*;
import java.net.*;
import java.util.concurrent.BlockingQueue; 
/**
 * This thread is responsible to handle client connection.
 *
 * @author www.codejava.net
 */
public class ServerThread extends Thread {
    private Socket socket;
    private static String internalText="";
    private final BlockingQueue queue;
 
    public ServerThread(Socket socket, BlockingQueue q) {
        this.socket = socket;
        this.queue=q;
        //this.internalText=string;
        //this.internalText=globalText;        
    }
    public static String getServerText(){
		return internalText;
	}	
    public static void resetServerText(){
		internalText="";
		return;
	}  
 
 
    public void run() {
        try {
			
            InputStream input = socket.getInputStream();
            BufferedReader reader = new BufferedReader(new InputStreamReader(input));
            //System.out.println("threadsetup starting"); 
            //OutputStream output = socket.getOutputStream();
            //PrintWriter writer = new PrintWriter(output, true);
            
 
            String text="";;
            
 
            do {
                text = reader.readLine();
                queue.add(text);                
			    } 
            while(!text.equals("byet")); 
            socket.close();			              
        } catch (IOException ex) {
            System.out.println("Server exceptional: " + ex.getMessage());
            ex.printStackTrace();
        }
    }
}
