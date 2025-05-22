module clock_divider (    
    input clk,
    input reset,
    output reg CLK250ms   // divided clock 
);
    reg [22:0] count;   // count is 23 bits 3shan ana gbt log2(6250000) >>> w tl3 22. ksor 

    always @(posedge clk or posedge reset) begin
        if (reset) begin    // hasfr kolo 
            count <= 0;
            CLK250ms <= 0;
        end
        else begin
            if (count < 6_250_000)     // f = 1/t     t=250ms ,,  0.25s   >> f =1/0.25=4hz >>  50000000(hz of fpga)/4 =12500000 >> /2 = 6250000 
                count <= count + 1;
            else begin
                CLK250ms <= ~CLK250ms;  // wahda high w wahda low 
                count <= 0;  // start count b zero 
            end
        end
    end
endmodule
module ALU (
    input [31:0] A, B,
    input [2:0] ALU_sel,    // input selector f 3 bits 
    output reg [31:0] ALU_result   // el result hytl3 f 32 bit 
);

    always @(*) begin   // combinational circuit 
        case (ALU_sel)
            3'b000: ALU_result = A & B;    // AND
            3'b001: ALU_result = A | B;    // OR
            3'b010: ALU_result = A + B;    // ADD
            3'b011: ALU_result = A - B;    // SUBTRACT  >> 
            3'b100: ALU_result = A * B;   // MULTIPLY
            3'b101: ALU_result = (A > B);  // GREATER THAN
            3'b110: ALU_result = (A < B);  // LESS THAN
            3'b111: ALU_result = (A == B); // EQUAL
            default: ALU_result = 0;       // DEFAULT CASE
        endcase
    end
endmodule

module car_counter (   
    input wire clk,
    input wire rst,
    input wire car_entry_pulse,
    input wire car_exit_pulse,
    output reg [1:0] car_count,  // car count can enter hyzed b wahed car 5rgt hy2l b wahed 
    output wire parking_full,
    output wire parking_empty
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            car_count <= 2'b00;
        end
        else begin
            if (car_entry_pulse && car_count < 2'b11) 
                car_count <= car_count + 1'b1;
           
            else if (car_exit_pulse && car_count > 2'b00) 
                car_count <= car_count - 1'b1;
                    end
    end
    assign parking_full = (car_count == 2'b11);
    assign parking_empty = (car_count == 2'b00);

endmodule
module timer_counter (   // dah el byhsb el wa2t ll3rbya d5l emta w 5rg emta 
    input wire clk,
    input wire rst,
    output reg [31:0] timer_count
);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            timer_count <= 32'b0;
        end
        else begin
            timer_count <= timer_count + 1'b1;  //123456789  9-5
        end
    end

endmodule
module car_buffer (  // el by save el 3rbya d5lt emta w 5rgt emta 
    input wire clk,
    input wire rst,
    input wire store_entry,
    input wire [1:0] car_id,
    input wire [31:0] entry_time,
    output reg [31:0] exit_entry_time
);

    reg [31:0] entry_time_buffer [2:0];  // Buffer for 3 cars (0-2)>>[2:0]  dol bto3 el buffer buf [0]

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all buffer entries and output
            entry_time_buffer[0] <= 32'd0;
            entry_time_buffer[1] <= 32'd0;
            entry_time_buffer[2] <= 32'd0;
            exit_entry_time <= 32'd0;
        end
        else begin
            if (store_entry) begin   // if the car enter 
                // Store entry time for the specified car_id
                entry_time_buffer[car_id] <= entry_time;   // ana b3ml save bl car id
            end
            // Output the entry time for the specified car_id
            exit_entry_time <= entry_time_buffer[car_id];
        end
    end

endmodule
module cost_calculator (
    input wire clk,
    input wire rst,
    input wire calculate,
    input wire [31:0] entry_time,
    input wire [31:0] exit_time,
    output reg [31:0] cost
);

    wire [31:0] alu_result;  /// el result hattl3 f 32 bits
    reg [2:0] alu_sel;

    ALU alu_inst (
        .A(exit_time),   // d5li mkan el A el exit time 
        .B(entry_time),
        .ALU_sel(alu_sel),
        .ALU_result(alu_result)
    );

    always @(posedge clk or posedge rst) begin
        if (rst) begin 
            cost <= 32'd0;
            alu_sel <= 3'b011;  // Default to subtraction operation
        end
        else if (calculate) begin
            alu_sel <= 3'b011;  // Subtraction operation (exit_time - entry_time)
            cost <= alu_result; // Store the time difference as cost
        end
    end

endmodule
module parking_fsm (  // el fsm el byb3t kol el signals 
    input wire clk,
    input wire rst,
    input wire car_entry,
    input wire car_exit,
    input wire [1:0] car_id,
    input wire [31:0] current_time,
    input wire parking_full,
    input wire parking_empty,
    input wire [31:0] stored_entry_time,
    output reg car_entry_pulse,
    output reg car_exit_pulse,
    output reg store_entry,
    output reg calculate,
    output reg [31:0] exit_time,
    output reg [1:0] state  // 2 bits 
);

    // State definitions
    parameter IDLE      = 2'b00;
    parameter ENTRY     = 2'b01;
    parameter EXIT      = 2'b10;
    parameter CALCULATE = 2'b11;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all outputs and state
            state <= IDLE;  // if rst back to IDLE state w kol el pulses b zero el hya el signals 
            car_entry_pulse <= 1'b0;
            car_exit_pulse <= 1'b0;
            store_entry <= 1'b0;
            calculate <= 1'b0;
            exit_time <= 32'd0;
        end
        else begin
            // Default outputs  >> el default bta3 el signals b zero 
            car_entry_pulse <= 1'b0;
            car_exit_pulse <= 1'b0;
            store_entry <= 1'b0;
            calculate <= 1'b0;

            // State transitions and outputs
            case (state)
                IDLE: begin
                    if (!car_entry && !parking_full) begin
                        state <= ENTRY; // go from IDLE to state ENERY 
                    end
                    else if (!car_exit && !parking_empty) begin
                        state <= EXIT;  // haro7 mn idle l exit 
                    end
                end

                ENTRY: begin   // lma car td5ol yb3t signal bwahed 
                    car_entry_pulse <= 1'b1;
                    store_entry <= 1'b1;  // yb3t signal l system y2olo save time el car d5lt feh 
                    state <= IDLE;  // b3den yrg3 tani l idle 
                end

                EXIT: begin 
                    car_exit_pulse <= 1'b1;
                    exit_time <= current_time;
                    state <= CALCULATE;
                end

                CALCULATE: begin
                    calculate <= 1'b1;  // send signal to the system eno y3ml calculation 
                    state <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
module parking_system_top (  //  el top feh kol el instanctions btrkb kol haga f b3deha >> el fsm byb3t signals lma 7aga t7sl yb3t signal 3shan y3rf eni dost push bottun msln ama el top enta btwsl el hagat bb3dha 
    input wire clk,  // el top akno el twselat wel fsm aknk wslt el khrba 
    input wire rst,
    input wire car_entry,
    input wire car_exit,
    input wire [1:0] car_id,
    output wire [1:0] car_count,
    output wire [31:0] cost,
    output wire parking_full,
    output wire parking_empty
);

    // Internal signals
    wire car_entry_pulse;
    wire car_exit_pulse;
    wire store_entry;
    wire calculate;
    wire [31:0] current_time;
    wire [31:0] stored_entry_time;
    wire [31:0] exit_time;
    wire clk_250ms;

    // Clock divider instance
    clock_divider clk_div_inst (
        .clk(clk),
        .reset(rst),
        .CLK250ms(clk_250ms)
    );

    // Timer counter instance
    timer_counter timer_inst (
        .clk(clk),
        .rst(rst),
        .timer_count(current_time)
    );

    // Car counter instance
    car_counter counter_inst (
        .clk(clk),
        .rst(rst),
        .car_entry_pulse(car_entry_pulse),
        .car_exit_pulse(car_exit_pulse),
        .car_count(car_count),
        .parking_full(parking_full),
        .parking_empty(parking_empty)
    );

    // Car buffer instance
    car_buffer buffer_inst (
        .clk(clk),
        .rst(rst),
        .store_entry(store_entry),
        .car_id(car_id),
        .entry_time(current_time),
        .exit_entry_time(stored_entry_time)
    );

    // Cost calculator instance
    cost_calculator calc_inst (
        .clk(clk),
        .rst(rst),
        .calculate(calculate),
        .entry_time(stored_entry_time),
        .exit_time(exit_time),
        .cost(cost)
    );

    // Parking FSM instance
    parking_fsm fsm_inst (
        .clk(clk),
        .rst(rst),
        .car_entry(car_entry),
        .car_exit(car_exit),
        .car_id(car_id),
        .current_time(current_time),
        .parking_full(parking_full),
        .parking_empty(parking_empty),
        .stored_entry_time(stored_entry_time),
        .car_entry_pulse(car_entry_pulse),
        .car_exit_pulse(car_exit_pulse),
        .store_entry(store_entry),
        .calculate(calculate),
        .exit_time(exit_time)
    );

endmodule
 

module parking_system_tb;

    // Inputs
    reg clk;
    reg rst;
    reg car_entry;
    reg car_exit;
    reg [1:0] car_id;

    // Outputs
    wire [1:0] car_count;
    wire [31:0] cost;
    wire parking_full;
    wire parking_empty;

    // Instantiate the Unit Under Test (UUT)
    parking_system_top uut (
        .clk(clk),
        .rst(rst),
        .car_entry(car_entry),
        .car_exit(car_exit),
        .car_id(car_id),
        .car_count(car_count),
        .cost(cost),
        .parking_full(parking_full),
        .parking_empty(parking_empty)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;    // forever tfdl mkmla 
    end

    // Test sequence
    initial begin
        // Initialize Inputs
        rst = 1;  // el rst 3andi b wahed ga hasfr kolo
        car_entry = 1;
        car_exit = 1;
        car_id = 0;

        // Reset the system
        #20 rst = 0;

        // Test car entries
        #20;
        car_id = 2'b00;
        #20 car_entry = 0;   // car entry b active low 
        #20 car_entry = 1;

        #100;

        car_id = 2'b01;
        #20 car_entry = 0;
        #20 car_entry = 1;

        #100;

        car_id = 2'b10;
        #20 car_entry = 0;
        #20 car_entry = 1;

        #100;

         car_id = 2'b11;
        #20 car_entry = 0;
        #20 car_entry = 1;

        #100;


        // Test car exits
        car_id = 2'b01;
        #20 car_exit = 0;
        #20 car_exit = 1;

        #100;

        car_id = 2'b00;
        #20 car_exit = 0;
        #20 car_exit = 1;

        #100;

        car_id = 2'b10;
        #20 car_exit = 0;
        #20 car_exit = 1;

        #100;

        car_id = 2'b00;
        #20 car_exit = 0;
        #20 car_exit = 1;

        #100;

        // End simulation
        $finish;
    end

endmodule




