`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/02 14:45:14
// Design Name: 
// Module Name: commit_ssd_detector
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module commit_ssd_detector (
    input wire clk,             // システムクロック (125MHz)
    input wire rst_n,           // リセット信号 (Active Low)
    input wire enable,          // 動作許可信号 (ウィンドウ制御用)
    input wire data_in,         // デコード済みデータ (1 bit)
    input wire valid_in,        // データ有効フラグ
    output reg detect_out,      // 検知パルス (SSD検知 + 660bit待機後にHigh)
    output wire tracking_active // ステータス信号: パターン追跡中または遅延待機中
);

    // =========================================================
    // 1. パラメータ定義
    // =========================================================
    // 同期パターン "00011"
    localparam [4:0] PAT_SYNC = 5'b00011;
    // トリガー(SSD)パターン "00100"
    localparam [4:0] PAT_TRIG = 5'b00100;

    // 遅延設定: commitとssdを検知後に出力を遅らせる時間
    // 60 bits * 10 clocks/bit = 600 clocks(プリアンブル)
    localparam integer DELAY_BITS   = 60;
    localparam integer CLKS_PER_BIT = 10;
    localparam integer DELAY_CYCLES = DELAY_BITS * CLKS_PER_BIT;

    // =========================================================
    // 2. ステートマシンの状態定義
    // =========================================================
    localparam [1:0] S_SEARCH    = 2'd0; // 探索
    localparam [1:0] S_SYNC_LOOP = 2'd1; // 同期中
    localparam [1:0] S_CHECK_2ND = 2'd2; // 確認

    // =========================================================
    // 3. 内部信号定義
    // =========================================================
    reg [1:0] state;         // ステートレジスタ
    reg [4:0] shift_reg;     // 受信データ履歴
    reg [2:0] bit_cnt;       // ワード境界カウンタ

    // 遅延制御用
    // 変更: 6600を数えるため 13ビット幅 (max 8191) に拡張
    reg [12:0] delay_cnt;
    reg        delay_running; // 遅延タイマー動作中フラグ

    // パターン照合用の先読みワイヤ
    wire [4:0] current_pattern;
    assign current_pattern = {shift_reg[3:0], data_in};

    // ---------------------------------------------------------
    // 追跡中ステータス (Topモジュールへの通知)
    // ---------------------------------------------------------
    assign tracking_active = (state != S_SEARCH) || delay_running;

    // =========================================================
    // 4. メインロジック
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 非同期リセット
            state         <= S_SEARCH;
            shift_reg     <= 5'd0;
            bit_cnt       <= 3'd0;
            detect_out    <= 1'b0;
            delay_cnt     <= 13'd0; // リセット値も13bit
            delay_running <= 1'b0;
        end else begin
            // デフォルト: 検知パルスは1クロック幅
            detect_out <= 1'b0;

            // --- Enable制御 ---
            if (!enable) begin
                state         <= S_SEARCH;
                bit_cnt       <= 3'd0;
                delay_running <= 1'b0;
                delay_cnt     <= 13'd0;
            end
            else begin
                // ------------------------------------------------
                // A. 遅延カウンタ処理 (優先度: 高)
                // ------------------------------------------------
                if (delay_running) begin
                    if (delay_cnt == DELAY_CYCLES - 1) begin
                        // 660ビット待機完了 -> ここで初めてパルスを出力
                        detect_out    <= 1'b1;
                        delay_running <= 1'b0; // タイマー停止
                    end else begin
                        delay_cnt <= delay_cnt + 1'b1;
                    end
                end

                // ------------------------------------------------
                // B. パターン検知処理 (データ有効時のみ動作)
                // ------------------------------------------------
                if (valid_in) begin
                    // シフトレジスタ更新
                    shift_reg <= current_pattern;

                    case (state)
                        // --- 1. 探索モード ---
                        S_SEARCH: begin
                            if (current_pattern == PAT_SYNC) begin
                                state   <= S_SYNC_LOOP;
                                bit_cnt <= 3'd0;
                            end
                        end

                        // --- 2. 同期維持・監視モード ---
                        S_SYNC_LOOP: begin
                            if (bit_cnt == 3'd4) begin
                                if (current_pattern == PAT_SYNC) begin
                                    state <= S_SYNC_LOOP;
                                end else if (current_pattern == PAT_TRIG) begin
                                    state <= S_CHECK_2ND;
                                end else begin
                                    state <= S_SEARCH;
                                end
                                bit_cnt <= 3'd0;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end

                        // --- 3. 確定チェックモード ---
                        S_CHECK_2ND: begin
                            if (bit_cnt == 3'd4) begin
                                if (current_pattern == PAT_TRIG) begin
                                    // "00100" (2回目) -> 検知成功！

                                    // 直ちに出力せず、遅延タイマーを起動する
                                    delay_running <= 1'b1;
                                    delay_cnt     <= 13'd0; // リセット

                                    // FSMは探索モードへ戻る
                                    state <= S_SEARCH;

                                end else if (current_pattern == PAT_SYNC) begin
                                    state <= S_SYNC_LOOP;
                                end else begin
                                    state <= S_SEARCH;
                                end
                                bit_cnt <= 3'd0;
                            end else begin
                                bit_cnt <= bit_cnt + 1'b1;
                            end
                        end
                    endcase
                end
            end
        end
    end
endmodule
