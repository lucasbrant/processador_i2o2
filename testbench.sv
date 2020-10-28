module test();
  reg clk, rst;
  wire [31:0] out;
  
  pipemips mips(clk, rst, out);
  
  initial begin
    
    $monitor("time = %g, test = %d ", $time, out);
    clk = 1'b1;
    rst = 1'b0;      
     
  end

   always begin
    #1 clk = !clk;
    #100 rst =~rst;
   end
   
   initial
    #3000 $finish;
endmodule