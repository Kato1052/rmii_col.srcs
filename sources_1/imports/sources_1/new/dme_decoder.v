`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2026/01/02 14:45:14
// Design Name: 
// Module Name: dme_decoder
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


module dme_decoder (
    input wire clk,          // システムクロック (125MHz)
    input wire rst_n,        // リセット信号 (Active Low)
    input wire dme_in,       // 差動マンチェスタ入力信号 (12.5MHz)
    output reg data_out,     // デコード済みデータ (1 bit)
    output reg valid_out,    // データ有効パルス (1クロック幅)
    output reg error_out     // タイミングエラー検知フラグ
);

    // =========================================================
    // 1. パラメータ定義 (パルス幅の判定基準)
    // =========================================================
    // 基準: 125MHz / 12.5MHz = 10クロック (1ビットあたりの長さ)
    // 差動マンチェスタ符号の特性:
    //  - "0": ビット中央で遷移しない (エッジ間隔 = 1ビット幅 = 10clk)
    //  - "1": ビット中央で遷移する   (エッジ間隔 = 0.5ビット幅 = 5clk が2回)

    // Shortパルス (データ"1"の構成要素, T/2)
    // 理想値: 5clk -> 許容範囲: 3〜7clk
    localparam CNT_SHORT_MIN = 4'd3;
    localparam CNT_SHORT_MAX = 4'd7;

    // Longパルス (データ"0", T)
    // 理想値: 10clk -> 許容範囲: 8〜13clk
    localparam CNT_LONG_MIN  = 4'd8;
    localparam CNT_LONG_MAX  = 4'd13;

    // タイムアウト (信号断検知用)
    // これ以上エッジが来なければ無信号とみなす
    localparam CNT_TIMEOUT   = 4'd15;

    // =========================================================
    // 2. 内部信号定義
    // =========================================================
    reg [2:0] in_sync;       // 入力同期用シフトレジスタ (メタスタビリティ除去)
    reg [3:0] counter;       // エッジ間の時間を計測するカウンタ
    reg       half_bit_flag; // "Short"パルスの1回目を受信したことを示すフラグ
    wire      edge_detect;   // エッジ検出信号 (立ち上がり・立ち下がり両方)

    // =========================================================
    // 3. 入力同期とエッジ検出
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_sync <= 3'b000;
        end else begin
            // 外部信号を内部クロックに同期させる (2段FF + エッジ検出用1段)
            in_sync <= {in_sync[1:0], dme_in};
        end
    end

    // 現在の値(in_sync[1])と1クロック前の値(in_sync[2])が異なればエッジと判定
    // XORを使うことで、0->1(立ち上がり) と 1->0(立ち下がり) の両方を検出
    assign edge_detect = (in_sync[1] ^ in_sync[2]);

    // =========================================================
    // 4. 計測・デコードロジック
    // =========================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            counter       <= 4'd0;
            data_out      <= 1'b0;
            valid_out     <= 1'b0;
            error_out     <= 1'b0;
            half_bit_flag <= 1'b0;
        end else begin
            // デフォルト動作: 出力パルスは1クロックのみ
            valid_out <= 1'b0;
            error_out <= 1'b0;

            if (edge_detect) begin
                // --- エッジ検出時の処理 (計測終了・判定) ---

                // ケースA: 長いパルス ("0" を検出)
                // 前のエッジから 8〜13クロック経過している場合
                if (counter >= CNT_LONG_MIN && counter <= CNT_LONG_MAX) begin
                    if (half_bit_flag) begin
                        // エラー: "Short"が1回来て待機中なのに"Long"が来た
                        // (マンチェスタ符号の則に反する)
                        error_out <= 1'b1;
                        half_bit_flag <= 1'b0; // リセット
                    end else begin
                        // 正常: データ "0" を確定
                        data_out  <= 1'b0;
                        valid_out <= 1'b1;
                    end
                end

                // ケースB: 短いパルス ("1" の前半または後半)
                // 前のエッジから 3〜7クロック経過している場合
                else if (counter >= CNT_SHORT_MIN && counter <= CNT_SHORT_MAX) begin
                    if (half_bit_flag) begin
                        // 2回目のShortパルス受信 -> これでデータ "1" が完成
                        data_out      <= 1'b1;
                        valid_out     <= 1'b1;
                        half_bit_flag <= 1'b0; // 処理完了につきフラグクリア
                    end else begin
                        // 1回目のShortパルス受信 -> まだデータ確定ではない
                        // 次のShortパルスを待つためにフラグをセット
                        half_bit_flag <= 1'b1;
                    end
                end

                // ケースC: 規定外のパルス幅（ノイズなど）
                else begin
                    error_out <= 1'b1;
                    half_bit_flag <= 1'b0; // 安全のためリセット
                end

                // 計測完了のためカウンタをリセット
                counter <= 4'd0;

            end else begin
                // --- エッジが来ない間の処理 (計測中) ---
                if (counter < CNT_TIMEOUT) begin
                    counter <= counter + 1'b1;
                end else begin
                    // タイムアウト（一定期間信号変化なし -> アイドル状態）
                    // 途中で信号が途切れた場合のゴミデータを防ぐためフラグをクリア
                    half_bit_flag <= 1'b0;
                end
            end
        end
    end

endmodule
