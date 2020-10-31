// Code your design here

//sufixo op1 -> variaveis relacionadas ao operando 1
//sufixo op2 -> variaveis relacionadas ao operando 2
//sufixo dest -> variaveis relacionadas ao reg de destino

module scoreboard(input clk, new_line, input[4:0] id_op1, id_op2, id_dest, input[6:0] op_code, output reg pendencia_op1, pendencia_op2, pendencia_dest, output reg[1:0] unit_op1, unit_op2, unit_dest);
  
  integer i;
  reg[7:0] score[0:31]; // 32 linhas de 8 bits cada

  // fill the scoreboard
  initial begin
    for (i = 0; i <= 31; i++) 
      score[i] <= 8'b0;     
  end
  
  //TESTES
  /*initial begin
    
    //reg1 esta pendente, no ultimo estagio da unidade funcional '00'
    score[1] = 8'b10000001;
  
    //reg2 esta pendente, no penultimo estagio da unidade funcional '01'
    score[2] = 8'b10100010;
    
  end */
  
  //precisamos fazer 2 coisas em cada ciclo, atualizar as entradas atuais e inserir a nova entrada (se houver)
  
  //atualiza entradas no scoreboard
  always@(posedge clk) begin
    
    //$monitor("time = %g, linha_1 = %b", $time, score[2]);
  	
    //atualiza os valores no scoreboard atual
    for (i = 0; i <= 31; i++)
      	//se temos o ultimo bit como 1 o dado ja foi escrito
      	if(score[i][0] == 1) 
          score[i] <= 8'b0;
    	else
          score[i] <= {score[i][7:5], score[i][4:0] >> 1};
    
    pendencia_op1 <= score[id_op1][7];
    pendencia_op2 <= score[id_op2][7];
    pendencia_dest <= score[id_dest][7];
    
    unit_op1 <= score[id_op1][6:5];
    unit_op2 <= score[id_op2][6:5];
    unit_dest <= score[id_dest][6:5];
    
  end
  
  //insere nova entrada no scoreboard
  always @(new_line) begin
    if(new_line == 1)
      case(op_code)
        7'b0110011: begin	// R type == 51 usa X
          score[id_dest] <= 8'b10100001;
        end

        7'b1100011: begin // beq == 99 usa X
          score[id_dest] <= 8'b10100001;
        end

        7'b0010011: begin // addi == 19 usa X
          score[id_dest] <= 8'b10100001;
        end

        7'b0000011: begin // lw == 3 usa M
          score[id_dest] <= 8'b11000010;
        end

        7'b0100011: begin // sw == 35 usa M
          score[id_dest] <= 8'b11000010;
        end

        default: begin
          score[id_dest] <= 8'b0;
        end

      endcase
  end
  
endmodule
      
  
  