// Your code

module RISCV(clk,
            rst_n,
            // for mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // for mem_I
            mem_addr_I,
            mem_rdata_I
    );

    input             clk, rst_n ;
    // for mem_D
    output reg        mem_wen_D  ;  // mem_wen_D is high, CHIP writes data to D-mem; else, CHIP reads data from D-mem
    output reg [29:0] mem_addr_D ;  // the specific address to fetch/store data 
    output reg [63:0] mem_wdata_D;  // data writing to D-mem 
    input      [63:0] mem_rdata_D;  // data reading from D-mem
    // for mem_I
    output reg [29:0] mem_addr_I ;  // the fetching address of next instruction
    input      [31:0] mem_rdata_I;  // instruction reading from I-mem
    
    // i/o of mem_I
    reg    [31:0] Instruction;
    //output of mem_D
    reg    [63:0] Memory_read;
    //PC_generator
    reg    [31:0] PC_nxt;
    reg    [31:0] PC;
    // register
    reg    [63:0] Write_data_nxt;
    reg           RegWrite;
    reg    [63:0] Read_data_1;
    reg    [63:0] Read_data_2;
    wire    [63:0] Read_data_1_wire;
    wire    [63:0] Read_data_2_wire;
    //Alu
    reg    [3:0]  Aluop;
    reg           Alusrc1;
	reg    [1:0]  Alusrc2;
    reg    [63:0] Imm_gen;
    reg    [63:0] Alu_result;
    //Memory
    reg           Memwrite;
    //Write_data_nxt_generator
    reg    [1:0]  Memtoreg;
    //control
    reg           Branch;
    reg           Jal;
    reg           Jalr;
	
	reg        [4:0]  opcode;
	reg [4:0] signal;
   
    always@(*)
    begin
		Memory_read={mem_rdata_D[7:0],mem_rdata_D[15:8],mem_rdata_D[23:16],mem_rdata_D[31:24],mem_rdata_D[39:32],mem_rdata_D[47:40],mem_rdata_D[55:48],mem_rdata_D[63:56]};
        Instruction={mem_rdata_I[7:0],mem_rdata_I[15:8],mem_rdata_I[23:16],mem_rdata_I[31:24]};
        mem_addr_I=PC[31:2];
        mem_wen_D=Memwrite;
        mem_addr_D=Alu_result[31:2];
		mem_wdata_D={Read_data_2[7:0],Read_data_2[15:8],Read_data_2[23:16],Read_data_2[31:24],Read_data_2[39:32],Read_data_2[47:40],Read_data_2[55:48],Read_data_2[63:56]};
        Read_data_1=Read_data_1_wire;
        Read_data_2=Read_data_2_wire;
		opcode=Instruction[6:2];
           
    end
    
    //PC_generator 
    always@(posedge clk)
    begin
        if(!rst_n)
            PC<=0;
        else
            PC<=PC_nxt;
    end     
    

    register r(.Clk(clk),
               .rst_n(rst_n),
               .Read_register_1(Instruction[19:15]),
               .Read_register_2(Instruction[24:20]),
               .Write_reg(Instruction[11:7]),
               .Write_data(Write_data_nxt),
               .RegWrite(RegWrite),
               .Read_data_1(Read_data_1_wire),
               .Read_data_2(Read_data_2_wire)
    );
    
    
    //Alu A
    
    reg                 carry;
	reg          [63:0] input1;
    reg          [63:0] input2;
    reg          [63:0] sub_result;
	reg                 zero;
    always@(*)
    begin
		if(Alusrc1)
		begin
			input1={32'd0,PC};
		end
		else
		begin
			input1=Read_data_1;
		end
		
		case(Alusrc2)
			0:input2=Read_data_2;
			1:input2=Imm_gen;
			2:input2=64'd4;
			default:input2=Read_data_2;
		endcase
		
        Alu_result=63'd0;
		zero=0;
        {carry, sub_result}=$signed(input1)-$signed(input2);   
        case(Aluop)
            0:{carry,Alu_result}=$signed(input1)+$signed(input2);
            1: Alu_result=sub_result;
            2: Alu_result=input1&input2;
            3: Alu_result=input1|input2;
            4: Alu_result=input1^input2;
            5: Alu_result=input1<<input2;
            6: Alu_result=$signed(input1)>>>$signed(input2);
            7: Alu_result=input1>>input2;
            8: Alu_result=sub_result[63];
            default:Alu_result=64'd0;
         endcase
		 
		if(signal[0])
		begin
			if(sub_result==0)
				zero=0;
			else
				zero=1;
		end
        else
		begin
			if(sub_result==0)
				zero=1;
			else
				zero=0;
        end
    end
 
          
           
    
   //Write_data_nxt_generator wd_n    
    always@(*)
    begin
        Write_data_nxt=64'd0;
        case(Memtoreg)
            0:Write_data_nxt=Alu_result;
            1:Write_data_nxt=Memory_read;
            default:Write_data_nxt=64'd0;
        endcase  
    end    
     
           
  //control crtl    
	parameter sig_jal=5'd0;
	parameter sig_jalr=5'd1;
	parameter sig_beq=5'd2;
	parameter sig_bne=5'd3;
	parameter sig_ld=5'd4;
	parameter sig_sd=5'd5;
	parameter sig_addi=5'd6;
	parameter sig_slti=5'd7;
	parameter sig_xori=5'd8;
	parameter sig_ori=5'd9;
	parameter sig_andi=5'd10;
	parameter sig_slli=5'd11;
	parameter sig_srli=5'd12;
	parameter sig_srai=5'd13;
	parameter sig_add=5'd14;
	parameter sig_sub=5'd15;
	parameter sig_sll=5'd16;//no
	parameter sig_slt=5'd17;
	parameter sig_xor=5'd18;
	parameter sig_srl=5'd19;//no
	parameter sig_sra=5'd20;//no
	parameter sig_or=5'd21;
	parameter sig_and=5'd22;
	parameter sig_wrong=5'd31;

	always@(*)
	begin
		if(opcode==5'b01100)//r type
		begin
			if(Instruction[30])
			begin
				if(Instruction[12])
				begin
					signal=sig_sra;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=1;
                    Alusrc1=0;
                    Alusrc2=0;
                    Aluop=4'd6;				
				end
				else
				begin
					signal=sig_sub;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=1;
                    Alusrc1=0;
                    Alusrc2=0;
                    Aluop=4'd1;	
				end
			end
			else
			begin
				case(Instruction[14:12])
					3'b000:
                    begin
                    signal=sig_add;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd0;	
                    end
					3'b001://signal=sig_sll;
                    begin
                        signal=sig_sll;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd5;				
                    end
					3'b010://signal=sig_slt;
                    begin
                        signal=sig_slt;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd8;				
                    end	
					3'b100://signal=sig_xor;
                    begin
                        signal=sig_xor;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd4;				
                    end	
					3'b101://signal=sig_srl;
                    begin
                        signal=sig_srl;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd7;			
                    end
					3'b110://signal=sig_or;
                    begin
                        signal=sig_or;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd3;				
                    end
					3'b111://signal=sig_and;
                    begin
                        signal=sig_and;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=1;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd2;				
                    end
					default://signal=sig_wrong;
                    begin
                        signal=sig_wrong;
                        Jal=0;
                        Jalr=0;
                        Branch=0;
                        Memwrite=0;
                        Memtoreg=0;
                        RegWrite=0;
                        Alusrc1=0;
                        Alusrc2=0;
                        Aluop=4'd0;
                    end
				endcase
			end
		end
		else if(opcode==5'b00100)
		begin
			case(Instruction[14:12])
				3'b000://signal=sig_addi;
                begin
                    signal=sig_addi;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=1;
                    Alusrc1=0;
                    Alusrc2=1;
                    Aluop=4'd0;			
                end
				3'b001://signal=sig_slli;
                begin
                    signal=sig_slli;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=1;
                    Alusrc1=0;
                    Alusrc2=1;
                    Aluop=4'd5;				
                end
				3'b010://signal=sig_slti;
                begin
                    signal=sig_slti;
				Jal=0;
				Jalr=0;
				Branch=0;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=1;
				Alusrc1=0;
				Alusrc2=1;
				Aluop=4'd8;			
			    end
				3'b100://signal=sig_xori;
                begin
                    signal=sig_xori;
				Jal=0;
				Jalr=0;
				Branch=0;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=1;
				Alusrc1=0;
				Alusrc2=1;
				Aluop=4'd4;			
			    end
				3'b101:
				begin
					if(Instruction[30])
						//signal=sig_srai;
                        begin
                            signal=sig_srai;
                            Jal=0;
                            Jalr=0;
                            Branch=0;
                            Memwrite=0;
                            Memtoreg=0;
                            RegWrite=1;
                            Alusrc1=0;
                            Alusrc2=1;
                            Aluop=4'd6;			
                        end
					else
						//signal=sig_srli;
                        begin
                            signal=sig_srli;
                            Jal=0;
                            Jalr=0;
                            Branch=0;
                            Memwrite=0;
                            Memtoreg=0;
                            RegWrite=1;
                            Alusrc1=0;
                            Alusrc2=1;
                            Aluop=4'd7;			
                        end
				end
				3'b110://signal=sig_ori;
                begin
                    signal=sig_ori;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=1;
                    Alusrc1=0;
                    Alusrc2=1;
                    Aluop=4'd3;			
                end
				3'b111://signal=sig_andi;
                begin
                    signal=sig_andi;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=1;
                    Alusrc1=0;
                    Alusrc2=1;
                    Aluop=4'd2;			
                end	
				default://signal=sig_wrong;
                begin
                    signal=sig_wrong;
                    Jal=0;
                    Jalr=0;
                    Branch=0;
                    Memwrite=0;
                    Memtoreg=0;
                    RegWrite=0;
                    Alusrc1=0;
                    Alusrc2=0;
                    Aluop=4'd0;
                end
			endcase				
		end
		else if(opcode==5'b11000)
		begin
			if(Instruction[12])
			begin
				//signal=sig_bne;
                signal=sig_bne;
                Jal=0;
				Jalr=0;
				Branch=1;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=0;
				Alusrc1=0;
				Alusrc2=0;
				Aluop=4'd0;	
			end
			else
			begin
				//signal=sig_beq;
                signal=sig_beq;
                Jal=0;
				Jalr=0;
				Branch=1;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=0;
				Alusrc1=0;
				Alusrc2=0;
				Aluop=4'd0;	
			end
		end
		else if(opcode==5'b00000)
		begin
			//signal=sig_ld;
            signal=sig_ld;
            Jal=0;
            Jalr=0;
            Branch=0;
            Memwrite=0;
            Memtoreg=1;
            RegWrite=1;
            Alusrc1=0;
            Alusrc2=1;
            Aluop=4'd0;	
		end	
		else if(opcode==5'b01000)
		begin
			//signal=sig_sd;
            signal=sig_sd;
            Jal=0;
				Jalr=0;
				Branch=0;
				Memwrite=1;
				Memtoreg=0;
				RegWrite=0;
				Alusrc1=0;
				Alusrc2=1;
				Aluop=4'd0;	
		end
		else if(opcode==5'b11011)
		begin
			//signal=sig_jal;
            signal=sig_jal;
            Jal=1;
				Jalr=0;
				Branch=0;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=1;
				Alusrc1=1;
				Alusrc2=2;
				Aluop=4'd0;	
		end
		else if(opcode==5'b11001)
		begin
			//signal=sig_jalr;
            signal=sig_jalr;
            Jal=0;
				Jalr=1;
				Branch=0;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=1;
				Alusrc1=1;
				Alusrc2=2;
				Aluop=4'd0;	
		end
		else
		begin
			//signal=sig_wrong;
            signal=sig_wrong;
            Jal=0;
				Jalr=0;
				Branch=0;
				Memwrite=0;
				Memtoreg=0;
				RegWrite=0;
				Alusrc1=0;
				Alusrc2=0;
				Aluop=4'd0;
		end
	end
   
   //Imm_gen   I
    always@(*)
    begin
        
        //I-type
        if(opcode==5'b00000||opcode==5'b11001||opcode==5'b00100)
        begin
			if(Aluop==6)
			begin
				Imm_gen={ 59'd0,Instruction[24:20] };
				
			end
			else
			begin
				Imm_gen={ {53{ Instruction[31]}},Instruction[30:25],Instruction[24:21],Instruction[20]};
				
			end
            
        end
        //S-type
        else if(opcode==5'b01000)
        begin
            Imm_gen={ {53{Instruction[31]}},Instruction[30:25],Instruction[11:8],Instruction[7]};
		
        end
        //B-type
        else if(opcode==5'b11000)
        begin
            Imm_gen={ {52{Instruction[31]}} ,Instruction[7],Instruction[30:25],Instruction[11:8],1'b0};
			
        end    
        //J-type
        else if(opcode==5'b11011)
        begin
            Imm_gen={ {44{Instruction[31]}} ,Instruction[19:12],Instruction[20],Instruction[30:25],Instruction[24:21],1'b0};
			
        end
        //R-type
        else 
        begin
            Imm_gen=64'd0;
			
        end          
    end 

    //PC_nxt_generator pc_n 
   
    always@(*)
    begin
        PC_nxt=32'd0;
        if(Jalr)
        begin
            PC_nxt=Read_data_1+Imm_gen;
        end
        else
        begin
            if(((Branch&& zero )||Jal))
                PC_nxt=PC+Imm_gen;
            else
                PC_nxt=PC+4;
        end
    end
    
endmodule

module register(Clk,
                rst_n,
                Read_register_1,
                Read_register_2,
                Write_reg,
                Write_data,
                RegWrite,
                Read_data_1,
                Read_data_2
    );
    input        Clk;
    input        rst_n;
    input [4:0]  Read_register_1;
    input [4:0]  Read_register_2;
    input [4:0]  Write_reg;
    input [63:0] Write_data;
    input        RegWrite;
    output reg [63:0]  Read_data_1;
    output reg [63:0]  Read_data_2;
    
    reg   [63:0] r [0:31] ;
    reg   [63:0] r_nxt [0:31] ;
    
    integer i; 
    
    always@(*)
    begin
        r[0]=63'd0;
        //output
        Read_data_1=r[Read_register_1];
        Read_data_2=r[Read_register_2];
        //r_nxt update
        for(i=1;i<32;i=i+1)
        begin
            if(Write_reg==i && RegWrite)
            begin
                r_nxt[i]=Write_data;
            end
            else
            begin
                r_nxt[i]=r[i];
            end
        end

    end
    
    always@(posedge Clk )
    begin
        if(!rst_n)
        begin
            for(i=1;i<32;i=i+1)
            begin
                r[i]<=63'd0;
            end
        end
        else
        begin
            for(i=1;i<32;i=i+1)
            begin
                r[i]<=r_nxt[i];
            end
        end

    end
    
endmodule






















