// @category Feedbot
import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.listing.Function;
import java.nio.file.*;
import java.util.*;
public class RetryFailed extends GhidraScript {
 public void run() throws Exception {
  String[] args=getScriptArgs();Path dir=Paths.get(args[0],currentProgram.getName());
  List<String> rows=Files.readAllLines(dir.resolve("index.tsv"));
  List<String> audit=new ArrayList<>();int seconds=Integer.parseInt(args[1]);
  DecompInterface di=new DecompInterface();di.openProgram(currentProgram);
  for(int i=1;i<rows.size();i++) {
   String[] r=rows.get(i).split("\t",-1);if(!r[2].equals("FAILED"))continue;
   Function f=getFunctionAt(toAddr(r[0]));DecompileResults result=di.decompileFunction(f,seconds,monitor);
   String error=result.getErrorMessage()==null?"":result.getErrorMessage().replace('\n',' ').replace('\t',' ');
   if(result.decompileCompleted()&&result.getDecompiledFunction()!=null) {
    String file=r[0]+".c";Files.writeString(dir.resolve(file),"/* AUTOMATIC DECOMPILER OUTPUT; not original source; not buildable firmware.\n * "+currentProgram.getName()+" @ "+r[0]+" : "+r[1]+"\n */\n"+result.getDecompiledFunction().getC());
    rows.set(i,r[0]+"\t"+r[1]+"\tok\t"+file+"\t"+error);audit.add(r[0]+"\tok\t"+seconds);
   } else audit.add(r[0]+"\tFAILED\t"+seconds+"\t"+error);
  }
  di.dispose();Files.write(dir.resolve("index.tsv"),rows);Files.write(dir.resolve("retry.tsv"),audit);
  int failed=0;for(String row:rows)if(row.contains("\tFAILED\t"))failed++;
  Path coverage=dir.resolve("coverage.txt");String text=Files.readString(coverage);
  text=text.replaceAll("(?m)^exported=.*$","exported="+(rows.size()-1-failed)).replaceAll("(?m)^failed=.*$","failed="+failed);
  Files.writeString(coverage,text);println("FEEDBOT_RETRY failed="+failed);
 }
}
