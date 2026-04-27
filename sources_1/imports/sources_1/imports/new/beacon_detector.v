`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/02 14:45:14
// Design Name: 
// Module Name: beacon_detector
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


module beacon_detector (
    input wire clk,          // システムクロック (125MHz)
    input wire rst_n,        // リセット信号 (Active Low)
    input wire data_in,      // デコード済みデータ (DMEデコーダより)
    input wire valid_in,     // データ有効フラグ (DMEデコーダより)
    output reg detect_out    // 検知パルス (00010が5回連続したらHigh)
);

    // =========================================================
    // 1. パラメータ定義
    // =========================================================
    // 検出したい5ビットのパターン ("00010")
    localparam [4:0] TARGET_PATTERN = 5'b00010;
    // 連続して検出する必要がある回数 (5回)
    localparam [2:0] TARGET_COUNT   = 3'd5;

    // =========================================================
    // 2. 内部信号定義
    // =========================================================
    reg [4:0] shift_reg;     // 受信データ履歴 (過去4ビット + 最新入力で判定)
    reg [2:0] bit_cnt;       // 5ビット区切り(ワード境界)管理用カウンタ
    reg [2:0] seq_cnt;       // パターンが何回連続したかを数えるカウンタ
    reg       is_aligned;    // 同期フラグ (1: 最初のパターンを発見し、5ビット周期で監視中)

    // =========================================================
    // 3. メインロジック
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 非同期リセット: 全レジスタを初期化
            shift_reg  <= 5'd0;
            bit_cnt    <= 3'd0;
            seq_cnt    <= 3'd0;
            is_aligned <= 1'b0;
            detect_out <= 1'b0;
        end else begin
            // デフォルト動作: 検知パルスは1クロック幅でリセット
            detect_out <= 1'b0;

            // デコーダから有効なビットが来た時のみ動作
            if (valid_in) begin
                // データを取り込んでシフトレジスタを更新
                shift_reg <= {shift_reg[3:0], data_in};

                // --- 判定ロジック ---
                // 現在のシフトレジスタ(過去4bit) + 最新入力(data_in) の5ビットを確認

                if (!is_aligned) begin
                    // ------------------------------------------------
                    // A. 探索モード (Search Mode)
                    // ------------------------------------------------
                    // まだ最初のパターンが見つかっていない状態。
                    // 1ビットずつずらしながら(スライディングウィンドウ)、"00010"を探す。

                    if ({shift_reg[3:0], data_in} == TARGET_PATTERN) begin
                        is_aligned <= 1'b1;   // 発見！ここを基準(アライメント)とする
                        seq_cnt    <= 3'd1;   // 1回目発見
                        bit_cnt    <= 3'd0;   // ビットカウンタをリセット
                    end
                end
                else begin
                    // ------------------------------------------------
                    // B. 検証モード (Check Mode)
                    // ------------------------------------------------
                    // 最初のパターンが見つかったので、5ビット間隔で連続性を確認する。

                    if (bit_cnt == 3'd4) begin
                        // 5ビット溜まったタイミングでチェック
                        if ({shift_reg[3:0], data_in} == TARGET_PATTERN) begin
                            // --- パターン一致 ---

                            if (seq_cnt == TARGET_COUNT - 1) begin
                                // 規定回数(5回)に到達した場合
                                detect_out <= 1'b1; // 検知成功パルスを出力

                                // 検知後は状態をリセットして次の探索へ
                                // (仕様によっては seq_cnt を0にするだけで同期維持する場合もあるが、
                                //  ここでは再探索のために同期を解除している)
                                seq_cnt    <= 3'd0;
                                is_aligned <= 1'b0;
                            end else begin
                                // まだ規定回数に満たない場合
                                seq_cnt <= seq_cnt + 1'b1; // カウントアップして継続
                            end
                        end else begin
                            // --- パターン不一致 ---
                            // 連続が途切れたため、同期を解除して最初から探し直す
                            is_aligned <= 1'b0;
                            seq_cnt    <= 3'd0;
                        end
                        bit_cnt <= 3'd0; // 5ビット数えたのでリセット
                    end else begin
                        // 5ビット溜まるまでカウントアップ
                        bit_cnt <= bit_cnt + 1'b1;
                    end
                end
            end
        end
    end

endmodule
