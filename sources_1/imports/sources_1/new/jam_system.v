`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/02 14:33:45
// Design Name: 
// Module Name: jam_system
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: ノードID 1のノードが送信した場合に、プリアンブルの後に無条件に32bit分妨害; 前のノードのCOMMITを検知してしまうバグあり;
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module jam_system (
    input wire sys_clk,       // システムクロック (125MHz)
    input wire sys_rst_n,     // リセット信号 (Active Low)
    input wire dme_rx_pin,    // FPGA入力ピン: 差動マンチェスタ信号 (12.5MHz)
    output reg jam_pin,       // FPGA出力ピン: ジャム信号出力 (640ビット長)
    output reg led_out        // 1000回実行したら点灯
);

    // =========================================================
    // 1. パラメータ定義 (時間設定)
    // =========================================================
    // 1ビットあたりのクロック数: 125MHz / 12.5MHz = 10クロック
    localparam integer CLKS_PER_BIT = 10;

    // [待機時間] 第1検知後、第2検知を開始するまでの待ち時間
    // 40 bits * 10 = 400 clocks (3.2us)
    localparam integer WAIT_CYCLES  = 40 * CLKS_PER_BIT;

    // [探索ウィンドウ] 第2検知を許可する基本期間
    // 40 bits * 10 = 400 clocks
    // ※ただし、検知器が「追跡中」の場合はこの期間を超えて延長される
    localparam integer HUNT_WINDOW  = 40 * CLKS_PER_BIT;

    // [出力時間] ジャム信号(High)を出力する期間
    // 40 bits * 10 = 400 clocks (32 * 8 * 1.25(32バイト * ビット * 4B5B))
    localparam integer OUT_CYCLES   = 320 * CLKS_PER_BIT;

    // =========================================================
    // 2. ステート定義
    // =========================================================
    localparam [1:0] ST_IDLE   = 2'd0; // 待機状態: 第1トリガー(Beacon)を待つ
    localparam [1:0] ST_WAIT   = 2'd1; // 遅延状態: 規定時間(40bit)待機する
    localparam [1:0] ST_HUNT   = 2'd2; // 探索状態: 第2トリガー(SSD)を探す (ウィンドウ制御)
    localparam [1:0] ST_OUTPUT = 2'd3; // 出力状態: ジャム信号を出力する

    // =========================================================
    // 3. 内部信号定義
    // =========================================================
    wire w_decoded_data;    // デコードされたビットデータ
    wire w_data_valid;      // データの有効フラグ
    wire w_beacon_pulse;    // 第1検知器からのパルス (Beacon検知)
    wire w_ssd_pulse;       // 第2検知器からのパルス (SSD検知 + 遅延後)
    wire w_is_tracking;     // 第2検知器が同期追跡中かどうかのフラグ

    reg [1:0]  state;       // ステートマシンの現在の状態
    reg [12:0] counter;     // 汎用タイマーカウンタ (6400を数えるため13bit必要)
    reg        ssd_enable;  // 第2検知器の動作許可信号 (Enable)
    reg [9:0]  jam_counter; // 妨害した回数

    // =========================================================
    // 4. サブモジュールの接続
    // =========================================================

    // --- 差動マンチェスタデコーダ ---
    // 物理層の信号をデジタルビット("0", "1")に変換します
    dme_decoder u_decoder (
        .clk       (sys_clk),
        .rst_n     (sys_rst_n),
        .dme_in    (dme_rx_pin),
        .data_out  (w_decoded_data),
        .valid_out (w_data_valid),
        .error_out () // エラーフラグはここでは未使用
    );

    // --- 第1検知器 (Beacon Detector) ---
    // 初期トリガーとなる "00010" の5回連続受信を監視します
    beacon_detector u_beacon_det (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .data_in    (w_decoded_data),
        .valid_in   (w_data_valid),
        .detect_out (w_beacon_pulse)
    );

    // --- 第2検知器 (Commit SSD Detector) ---
    // "00011..." から "00100"x2 への遷移を監視します
    // ※Enable信号により、特定の期間だけ動作させます
    commit_ssd_detector u_ssd_det (
        .clk        (sys_clk),
        .rst_n      (sys_rst_n),
        .enable     (ssd_enable),     // ステートマシンから制御
        .data_in    (w_decoded_data),
        .valid_in   (w_data_valid),
        .detect_out (w_ssd_pulse),    // 検知成功信号
        .tracking_active (w_is_tracking) // 同期ロック中信号
    );

    // =========================================================
    // 5. メインシーケンス制御ステートマシン
    // =========================================================
    always @(posedge sys_clk or negedge sys_rst_n) begin
        if (!sys_rst_n) begin
            state       <= ST_IDLE;
            counter     <= 13'd0;
            ssd_enable  <= 1'b0;
            jam_pin     <= 1'b0;
            jam_counter <= 10'd0;
            led_out     <= 1'b0;
        end else begin
            case (state)
                // ------------------------------------------------
                // ST_IDLE: 最初のきっかけ待ち
                // ------------------------------------------------
                ST_IDLE: begin
                    jam_pin    <= 1'b0; // 出力OFF
                    ssd_enable <= 1'b0; // 第2検知器OFF

                    // Beacon検知パルスが来たらシーケンス開始
                    if (w_beacon_pulse) begin
                        state   <= ST_WAIT;
                        counter <= 13'd0;
                    end
                end

                // ------------------------------------------------
                // ST_WAIT: 40ビット時間 (3.2us) の待機
                // ------------------------------------------------
                ST_WAIT: begin
                    if (counter == WAIT_CYCLES - 1) begin
                        state       <= ST_HUNT; // 次のフェーズへ
                        counter     <= 13'd0;   // カウンタリセット
                        ssd_enable  <= 1'b1;    // 第2検知器を有効化(ON)
                    end else begin
                        counter <= counter + 1'b1;
                    end
                end

                // ------------------------------------------------
                // ST_HUNT: 第2パターンの探索 (スマートウィンドウ)
                // ------------------------------------------------
                ST_HUNT: begin
                    // パターンA: 検知成功
                    if (w_ssd_pulse) begin
                        state       <= ST_OUTPUT; // 出力フェーズへ
                        counter     <= 13'd0;
                        ssd_enable  <= 1'b0;      // 検知器の役割終了

                        jam_counter <= jam_counter + 1'b1;

                        if (jam_counter == 10'd999) begin
                            led_out <= 1'b1; // 1000回目の出力でLED点灯
                        end
                    end
                    // パターンB: ウィンドウ時間終了時の判定
                    else if (counter >= HUNT_WINDOW - 1) begin
                        // ここで「まだ見込みがあるか？」を判定
                        if (w_is_tracking) begin
                            // 検知器が同期ロック中なら、タイムアウトさせずに待つ
                            counter     <= HUNT_WINDOW - 1; // カウンタを維持
                            ssd_enable  <= 1'b1;            // 検知器ON維持
                        end else begin
                            // 何もロックしていないなら、ただのタイムアウト -> 初期化
                            state       <= ST_IDLE;
                            counter     <= 13'd0;
                            ssd_enable  <= 1'b0;
                        end
                    end
                    // パターンC: ウィンドウ時間内
                    else begin
                        counter     <= counter + 1'b1;
                        ssd_enable  <= 1'b1;
                    end
                end

                // ------------------------------------------------
                // ST_OUTPUT: 640ビット時間のHigh出力 (Jamming)
                // ------------------------------------------------
                ST_OUTPUT: begin
                    jam_pin <= 1'b1; // 出力ピンをHighに

                    if (counter == OUT_CYCLES - 1) begin
                        // 規定時間経過したら終了
                        state   <= ST_IDLE; // 最初に戻る
                        jam_pin <= 1'b0;    // 出力をLowに
                    end else begin
                        counter <= counter + 1'b1;
                    end
                end
            endcase
        end
    end

endmodule
