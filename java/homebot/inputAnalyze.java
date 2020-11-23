//=======================================================================================
public class inputAnalyze
{
	boolean forwardStateOn=false; 
    boolean reverseStateOn=false;  
    boolean leftStateOn=false;
    boolean rightStateOn=false;
 public inputAnalyze(){}   
 //=======================================================================================
  public String inputAnalyze(String inputString,String inputJoyStick){
    String outputString=inputString;
    if (inputJoyStick!=null){
      int len=inputJoyStick.length();
      if (len>54){
		String shortString=inputJoyStick.substring(54,len);
        //System.out.println(shortString);
        switch(shortString)
        {		
		  case "Button 3 changed to On":		    
		    if (!forwardStateOn){				
				forwardStateOn=true;
			    outputString="w$";
			}
		    break;
		  case "Button 3 changed to Off":		   
		    if (forwardStateOn){
				forwardStateOn=false;
			    outputString="s$";
			}
		    break;
		  case "Button 0 changed to On":		    
		    if (!reverseStateOn){				
				reverseStateOn=true;
			    outputString="x$";
			}
		    break;
		  case "Button 0 changed to Off":		   
		    if (reverseStateOn){
				reverseStateOn=false;
			    outputString="s$";
			} 
		  case "Button 2 changed to On":		    
		    if (!leftStateOn){				
				leftStateOn=true;
			    //outputString="w$";
			}
		    break;
		  case "Button 2 changed to Off":		   
		    if (leftStateOn){
				leftStateOn=false;
			    outputString="a$";
			}
		    break;
		  case "Button 1 changed to On":		    
		    if (!rightStateOn){				
				rightStateOn=true;
			    //outputString="x$";
			}
		    break;
		  case "Button 1 changed to Off":		   
		    if (rightStateOn){
				rightStateOn=false;
			    outputString="s$";
			}  	   
		  default:
		    //outputString=inputString;
		  ;  
	    }
	  }
	}      	
	return outputString;
  }  
}  
