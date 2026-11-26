// Offline exports. Generated C is decompiler pseudocode, NOT compilable recovered source.
// @category Feedbot
import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.*;
import ghidra.program.model.listing.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.*;
import java.util.*;

public class ExportCore extends GhidraScript {
    boolean core(Function f, String mode) {
        String n=f.getName(true);
        if(mode.equals("all")) return true;
        for(String p:new String[]{"move_base::", "DockPerception::", "icp_2d::", "laser_processor::", "LinearPoseFilter2D::", "dwa_local_planner::", "base_local_planner::", "global_planner::", "rc_driver_ros::", "ltme", "LTME", "ldcp::"})
            if(n.contains(p.replace("::", ""))) return true;
        return n.equals("main");
    }
    public void run() throws Exception {
        String[] args=getScriptArgs();
        File root=new File(args[0],currentProgram.getName());root.mkdirs();
        String mode=args.length>1?args[1]:"core";
        Map<String,Function> selected=new TreeMap<>();
        Iterator<Function> it=currentProgram.getFunctionManager().getFunctions(true);
        int total=0;
        try(PrintWriter inventory=new PrintWriter(new File(root,"inventory.tsv"),"UTF-8")) {
            inventory.println("address\tname\texternal\tthunk\tselection");
            while(it.hasNext()) {
                Function f=it.next();total++;
                boolean take=!f.isExternal()&&!f.isThunk()&&core(f,mode);
                inventory.println(f.getEntryPoint()+"\t"+f.getName(true)+"\t"+f.isExternal()+"\t"+f.isThunk()+"\t"+(take?"export":"excluded"));
                if(take)selected.put(f.getEntryPoint().toString(),f);
            }
        }
        // Include recursively called unnamed internal helpers, not imported/runtime wrappers.
        ArrayDeque<Function> todo=new ArrayDeque<>(selected.values());
        while(!todo.isEmpty()) {
            Function f=todo.remove();
            for(Function c:f.getCalledFunctions(monitor)) {
                String key=c.getEntryPoint().toString();
                if(!c.isExternal()&&!c.isThunk()&&c.getName().startsWith("FUN_")&&!selected.containsKey(key)) {
                    selected.put(key,c);todo.add(c);
                }
            }
        }
        DecompInterface di=new DecompInterface();
        DecompileOptions opts=new DecompileOptions();di.setOptions(opts);
        di.toggleCCode(true);di.toggleSyntaxTree(false);di.openProgram(currentProgram);
        int ok=0,failed=0;
        try(PrintWriter idx=new PrintWriter(new File(root,"index.tsv"),"UTF-8");
            PrintWriter calls=new PrintWriter(new File(root,"calls.tsv"),"UTF-8")) {
            idx.println("address\tname\tstatus\tfile\terror");calls.println("caller\tcallee\tcallee_name");
            for(Function f:selected.values()) {
                if(monitor.isCancelled())break;
                String a=f.getEntryPoint().toString();
                for(Function c:f.getCalledFunctions(monitor))calls.println(a+"\t"+c.getEntryPoint()+"\t"+c.getName(true));
                DecompileResults r=di.decompileFunction(f,45,monitor);
                String error=r.getErrorMessage()==null?"":r.getErrorMessage().replace('\n',' ').replace('\t',' ');
                if(r.decompileCompleted()&&r.getDecompiledFunction()!=null) {
                    String name=a+".c";
                    String content="/* AUTOMATIC DECOMPILER OUTPUT; not original source; not buildable firmware.\n * "+currentProgram.getName()+" @ "+a+" : "+f.getName(true)+"\n */\n"+r.getDecompiledFunction().getC();
                    Files.write(new File(root,name).toPath(),content.getBytes(StandardCharsets.UTF_8));
                    idx.println(a+"\t"+f.getName(true)+"\tok\t"+name+"\t"+error);ok++;
                } else {idx.println(a+"\t"+f.getName(true)+"\tFAILED\t\t"+error);failed++;}
                idx.flush();calls.flush();
            }
        } finally {di.dispose();}
        Files.writeString(new File(root,"coverage.txt").toPath(),"image_base="+currentProgram.getImageBase()+"\nfunctions_detected="+total+"\nselected="+selected.size()+"\nexported="+ok+"\nfailed="+failed+"\nmode="+mode+"\n");
        println("FEEDBOT_EXPORT "+currentProgram.getName()+" selected="+selected.size()+" exported="+ok+" failed="+failed);
    }
}
