// @category Feedbot
import ghidra.app.script.GhidraScript;
import ghidra.program.model.address.Address;
import ghidra.program.model.lang.Register;
import java.math.BigInteger;
import java.util.*;
public class SeedThumb extends GhidraScript {
    public void run() throws Exception {
        Register t=currentProgram.getRegister("TMode");
        currentProgram.getProgramContext().setValue(t,toAddr(0x08005000L),toAddr(0x0800e76bL),BigInteger.ONE);
        Set<Long> entries=new TreeSet<>();
        for(long a=0x08005004L;a<0x08005100L;a+=4) {
            long v=Integer.toUnsignedLong(getInt(toAddr(a)));
            if((v&1)==1&&v>=0x08005100L&&v<0x0800e76cL)entries.add(v&~1L);
        }
        for(long v:new long[]{0x0800ce98L,0x0800cea8L,0x0800c1f8L,0x0800ceb6L,0x0800b7fcL,0x0800b0e8L,0x0800ab00L,0x0800ab84L,0x0800d170L,0x0800dee4L,0x0800b7c0L,0x0800d8d6L}) {
            // Last address is a known branch inside a function; do not invent a boundary there.
            if(v!=0x0800d8d6L)entries.add(v);
        }
        for(long v:entries) {
            Address a=toAddr(v); disassemble(a);
            if(getFunctionAt(a)==null)createFunction(a,null);
        }
        println("FEEDBOT_THUMB_SEEDS "+entries);
    }
}
