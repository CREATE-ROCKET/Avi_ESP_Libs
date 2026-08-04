//   ____    _    _   _    ____ ____  _____    _  _____ _____
//  / ___|  / \  | \ | |  / ___|  _ \| ____|  / \|_   _| ____|
// | |     / _ \ |  \| | | |   | |_) |  _|   / _ \ | | |  _|
// | |___ / ___ \| |\  | | |___|  _ <| |___ / ___ \| | | |___
//  \____/_/   \_\_| \_|  \____|_| \_\_____/_/   \_\_| |_____|
//
// ver 1.0.0
//
// Copyright (c) CREATE-ROCKET All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

/**
 * @file CANCREATE.h
 * @brief CANCREATE.cpp header
 * @copyright CREATE-ROCKET
 */

#pragma once

#ifndef CAN_H
#define CAN_H

#include "CANCREATE_lib.h"
#include "stdint.h"

/**
 * @class CAN_CREATE
 * @brief CANバスを制御するためのクラス
 */
class CAN_CREATE
{
private:
    twai_general_config_t _general_config;
    twai_filter_config_t _filter_config;
    twai_timing_config_t _timing_config;
    gpio_num_t _rx;
    gpio_num_t _tx;
    gpio_num_t _bus_off;
    uint32_t _id;
    bool _return_new;
    static bool _already_begin;
    bool _multi_send;
    can_setting_t _settings;

    int _begin(can_setting_t settings, twai_mode_t mode = TWAI_MODE_NORMAL);
    void _end();
    int _test(uint32_t id);
    int _sendLine(uint32_t id, char *data, int num, uint32_t waitTime);
    int _send(twai_message_t message, uint32_t waitTime);
    int _read(twai_message_t *message, uint32_t waitTime);
    void bus_on();
    void bus_off();
    int return_with_compatiblity(int return_int);

    friend void CanWatchDog(void *pvParameter);

public:
    /**
     * @brief CAN_CREATEのコンストラクタ
     *
     * @param[in] return_new 古いreturn手法を用いるか否か true推奨
     *             falseにした場合、setPin begin read sendPacket関数のみを用いる必要がある
     * @param[in] enableCanWatchDog Canが動作しなくなったら回復を試みるWatchDogを有効にする
     */
    CAN_CREATE(bool return_new = true, bool enableCanWatchDog = true);

    /**
     * @brief CAN_CREATE のデストラクタ end関数を呼び出しているだけ
     */
    ~CAN_CREATE();

    /**
     * @deprecated
     * 互換性のために残してある
     * 利用は非推奨
     */
    void setPins(int rx, int tx, uint32_t id = UINT32_MAX, int bus_off = GPIO_NUM_MAX);

    /**
     * @deprecated
     * 互換性のために残してある
     * 利用は非推奨
     */
    int begin(long baudRate);

    /**
     * @brief setup内で最初に実行するべき関数 一度だけ実行できる
     * @param[in] settings can_setting_t 型の引数 通信周波数等を設定可能 通信相手と共通化しておくこと
     * @param[in] rx rxピンを指定する。CANトランシーバーのrxピンに接続されているピンを指定すること
     * @param[in] tx txピンを指定する。CANトランシーバーのtxピンに接続されているピンを指定すること
     * @param[in] id 送信するときにデフォルトで利用するidを選択する 省略できるが、その場合送るときに逐一idを指定する必要がある
     * @param[in] bus_off CANトランシーバーのSTBYピンに接続されているピンを指定する なければ省略可
     *
     * @retval 0: success
     * @retval 2: 対応していない通信周波数を指定した
     * @retval 3: twai driverがインストールできなかった (txピン、rxピンに使用できないピンが指定されてる)
     * @retval 4: twai driverがstartできなかった (基本ないはず)
     * @retval 5: begin関数が再度呼び出された
     * @retval 6: bus_offピンにOUTPUTに指定できないピンが指定された
     * @retval 7: rxピンにOUTPUTに指定できないピンが指定された
     * @retval 8: txピンにOUTPUTに指定できないピンが指定された
     *
     * ```cpp
     * // example
     * can_setting_t can_setting = {
     *   ((long)100E3),
     *   false,
     *   CAN_FILTER_DEFAULT,
     * };
     *
     * CAN_CREATE CAN(true);
     *
     * void setup() {
     *     if(CAN.begin(100E3, 18, 19, 10, 20))
     *     {
     *         Serial.println("Starting CAN failed!!!");
     *         ESP.restart(); // CANが初期化できなかったときにESPを再起動する
     *     }
     * }
     * ```
     */
    int begin(can_setting_t settings, int rx, int tx, uint32_t id = UINT32_MAX, int bus_off = GPIO_NUM_MAX);

    /**
     * @brief setup内等で最初に実行するべき関数 一度だけのみ実行できる
     *
     * @param[in] baudRate 通信周波数 通信する相手と揃える必要がある
     * @param[in] rx rxピン
     * @param[in] tx txピン
     * @param[in] id CANの通信id (send時に逐一切り替えるなら空欄で良い)
     * @param[in] bus_off canの有効化と無効化を切り替えるピン(利用しなければ空欄で良い)
     *
     * @retval 0 success
     * @retval 2 対応していない通信周波数を指定した
     * @retval 3 twai driverがインストールできなかった (txピン、rxピンに使用できないピンが指定されてる)
     * @retval 4 twai driverがstartできなかった (基本ないはず)
     * @retval 5 begin関数が再度呼び出された
     * @retval 6 bus_offピンにOUTPUTに指定できないピンが指定された
     * @retval 7 rxピンにOUTPUTに指定できないピンが指定された
     * @retval 8 txピンにOUTPUTに指定できないピンが指定された
     *
     * ```cpp
     * // example
     * CAN_CREATE CAN(true);
     *
     * void setup() {
     *   if(CAN.begin(100E3, 18, 19, 10, 20))
     *   {
     *     Serial.println("Starting CAN failed!!!");
     *     ESP.restart(); // CANが初期化できなかったときにESPを再起動する
     *   }
     * }
     * ```
     */
    int begin(long baudRate, int rx, int tx, uint32_t id = UINT32_MAX, int bus_off = GPIO_NUM_MAX);

    /**
     * @brief CANバスを再設定する関数
     * @param settings CANバスの新しい設定
     * @param mode CANバスの新しいモード
     * @return 成功した場合は0、それ以外の場合はエラー
     */
    int re_configure(can_setting_t settings, twai_mode_t mode = TWAI_MODE_NORMAL);

    /**
     * @brief CANの利用終了時に呼び出されるべき関数
     *        基本使わなくて良い プログラムでこれ以上CANを使わないときに呼び出せる
     */
    void end();

    /**
     * @brief CANを一時停止させるときに利用できる関数
     * 送信も受信も行わなくていいときに利用できる
     */
    void suspend();

    /**
     * @brief CANの動作を再開させるときに利用する関数
     * suspend() でCANを停止させたあと再開させることができる
     */
    void resume();

    /**
     * @brief CANの送信ステータスを確認できる関数
     *
     * @return enumの can_errで定義されている
     * @retval can_err::CAN_SUCCESS: 前回の送信が正常に成功した
     * @retval can_err::CAN_NO_ALERTS: まだstatusが届いていない
     *              (send関数のすぐ後の配置などが要因でまだ送信中)
     * @retval can_err::CAN_BUS_ERROR: ACK ErrorやBit Errorなどのバスのエラー
     * @retval can_err::CAN_TX_FAILED: CAN_BUS_ERROR以外の理由でデータが送信できなかった
     * @retval can_err::CAN_UNKNOWN_ERROR: begin関数を実行していないまたは不明なエラー
     *
     * ```cpp
     * //example
     * if (CAN.sendChar('a')) {
     *   Serial.println("failed to send CAN data");
     * }
     *
     * delay(100); // CANライブラリが裏で処理を終わらせるのに必要な待ち時間 ステータスを見るためにある
     * switch (CAN.getStatus())
     * {
     *   case CAN_SUCCESS:
     *     Serial.println("Success to Send!!!");
     *     break;
     *   case CAN_NO_ALERTS:
     *     Serial.println("CREATE_CAN driver isn't done yet");
     *     break;
     *   case CAN_BUS_ERROR:
     *     Serial.println("Got a bus error on the CAN such as ACK Error");
     *     break;
     *   case CAN_TX_FAILED:
     *     Serial.println("Can't send data, something other than a bus error might wrong");
     *     break;
     *   default:
     *     Serial.println("Unknown error occurred!!!");
     *     break;
     *   }
     * ```
     */
    int getStatus();

    /**
     * @brief CANが動作するかを確かめる関数
     *
     * あくまで参考として利用してください
     * テストデータを送信して、Bus Errorとなった場合に自分が送ったデータを自分で読み取れるかを確認する
     *
     * @attention この関数は実行に0.2~3s程度かかるため、setup関数でのみ利用可能
     * @attention 実行中に送信されてきたメッセージは受け取れない可能性がある。
     * @attention お互いにtest関数を実行する場合、正しい結果が得られないことがある。
     * @note デフォルトでは通信id (1 << 11 - 1)で送信するため、このidはフィルタリングされる必要がある
     *          can_setting_tのfilter_configがデフォルト設定なら自動的に設定される
     *
     * @param[in] id テストで送信するidを指定する。空欄ならid 1 << 11 - 1 で送信される
     *            デフォルトの設定では1 << 11 - 1は事前に除外されているため CANのフィルタリング方法を知らないなら空欄のままが無難
     * @return CANの動作が正常に終了したかを確かめる
     * @retval CAN_SUCCESS: 正常終了
     * @retval CAN_UNKNOWN_ERROR: 失敗
     * @retval CAN_NO_RESPONSE_ERROR: 相手側のコントローラーかBUSが動いていないときのエラー
     * @retval CAN_CONTROLLER_ERROR: 自分のコントローラーが動いていないときのエラー
     *
     * ```cpp
     * //example
     * void setup() {
     *   ...
     *   // beginした後
     *   switch (CAN.test())
     *   {
     *   case CAN_SUCCESS:
     *     Serial.println("Success!!!");
     *     break;
     *   case CAN_UNKNOWN_ERROR:
     *     Serial.println("Unknown error occurred");
     *     break;
     *   case CAN_NO_RESPONSE_ERROR:
     *     Serial.println("No response error");
     *     break;
     *   case CAN_CONTROLLER_ERROR:
     *     Serial.println("CAN CONTROLLER ERROR");
     *     break;
     *   default:
     *     break;
     *   }
     * }
     * ```
     */
    int test(uint32_t id = (1 << 11) - 1);

    /**
     * @brief CANトランシーバーのキューを消去する
     * @warning 受信待ち、送信待ちのデータが消去されるため注意!!!
     */
    void flush();

    /**
     * @brief CANにデータが送られて来ているかを判別する関数
     *        戻り値が0でなければデータが送られてきており、read関数で読める
     *
     *
     * @retval 0    データが受信されていない or 受信エラー
     * @retval 1以上 データが受信キューにある
     *
     * @warning この関数は性質上エラーも0で返してしまうため注意
     *
     * ```cpp
     * // example
     * if (CAN.available()) {
     *   Serial.println("receiving data...");
     *   char Data;
     *   if (CAN.read(&Data)) {
     *     Serial.println("failed to get CAN data");
     *   }
     *   else
     *     Serial.printf("Can received!!!: %c\r\n", Data);
     * }
     * ```
     */
    int available();

    /**
     * @brief CANに送られてきているデータを読む関数 互換性のために残してある
     * @warning 読むのに失敗したときの挙動が安全でないため利用は非推奨
     */
    char read();

    /**
     * @brief CANに送られてきているデータを読む関数
     * 引数readDataに得られたデータが入る 複数文字が送られてきた場合は1文字のみを返す
     *
     * @warning available関数で1以上が帰ってきたときのみ利用可能
     * @warning また、1文字までしか対応しないため相手がsendPacket以外を利用した場合には最初の文字だけを返す
     *
     * @param[out] readData charのポインタ 成功時これにデータが入る
     *
     * @retval 0 success
     * @retval 1 CANに送られてきたデータを読むのに失敗した available関数を実行していない等
     * @retval 2~4 CANを読むときにエラーが発生した
     * @retval 5 得られたデータがISO 11898-1互換ではなかった
     * @retval 6 何も入っていないデータが得られた
     *
     * ```cpp
     * // example
     * if (CAN.available()) {
     *   Serial.println("receiving data...");
     *   char Data;
     *   if (CAN.read(&Data)) {
     *     Serial.println("failed to get CAN data");
     *   }
     *   else
     *     Serial.printf("Can received!!!: %c\n", Data);
     * }
     * ```
     */
    int read(char *readData, uint32_t waitTime = MAX_READ);

    /**
     * @brief CANに送られてきたデータを読む関数
     * CANでは8文字まで同時に送受信が可能 引数readDataに得られたデータが入り終端は0が入る
     *
     * @warning available 関数で1以上が帰ってきたときのみ利用可能
     *
     * @param[out] readData charの配列のポインタ 9文字は入れられるサイズが必要 これにデータが入る
     * @param[in] waitTime CANにデータが入っていなかったときにどのぐらい待つか デフォルトは空欄とすれば0msとなる
     *
     * @retval 0 success
     * @retval 1 CANに送られてきたデータを読むのに失敗した available関数を実行していない等
     * @retval 2~4 CANを読むときにエラーが発生した
     * @retval 5 得られたデータがISO 11898-1互換ではなかった
     * @retval 6 何も入っていないデータが得られた
     *
     * ```cpp
     * // example
     * if (CAN.available()) {
     *   Serial.println("receiving data...");
     *   char[9] Data; // 配列は9文字以上必要!!!
     *   if (CAN.readLine(Data)) {
     *     Serial.println("failed to get CAN data");
     *   }
     *   else
     *     Serial.printf("Can received!!!: %s\r\n", Data);
     * }
     * ```
     */
    int readLine(char *readData, uint32_t waitTime = MAX_READ);

    /**
     *  @brief CANのデータを読んで idや大きさも含まれる詳細なデータを返す関数
     *
     *  @param[out] readData 得られたデータを返す変数 詳細なデータも含まれているデータ型 can_return_t
     *
     *  @retval 0 success
     *  @retval 1 CANに送られてきたデータを読むのに失敗した available関数を実行していない等
     *  @retval 2~4 CANを読むときにエラーが発生した
     *
     *  @warning 返り値のdataフィールドはnull文字が最後に入っていないことに注意すること
     *
     * ```cpp
     * //example
     * if (CAN.available()) {
     *   Serial.println("receiving data...");
     *   can_return_t Data;
     *   if (CAN.readWithDetail(&Data)) {
     *     Serial.println("failed to get CAN data");
     *   }
     *   else {
     *      // 得られたデータを出力する %.*s フォーマット指定子は直前の引数を参照してその文字数だけ出力する
     *      Serial.printf("CAN received!!!\r\n id:\t %u \r\n size: \t %d \r\n data: \t %.*s\r\n",
     *                      Data.id,
     *                      Data.size,
     *                      Data.size,
     *                      Data.data
     *                   );
     *   }
     * }
     * ```
     */
    int readWithDetail(can_return_t *data, uint32_t waitTime = MAX_READ);

    /**
     * @deprecated
     * 互換性のために残してある
     * 利用は非推奨
     */
    uint8_t sendPacket(int id, char data);

    /**
     * @brief charをCANで送る関数 idを逐一指定する
     *
     * @param[in] id int型の送信するid
     * @param[in] data char型の送信するデータ
     *
     * @retval 0 success
     * @retval 2 不正なid idは11bitまで
     * @retval 3 データに誤りがある
     * @retval 4 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
     * @retval 5 twaiドライバが動作していない
     * @retval 6 unknown error
     *
     * ```cpp
     * // example
     * if (Serial.available()) {
     *   char cmd = Serial.read()
     *   Serial.println(cmd);
     *   if (CAN.sendChar(10, cmd)) {
     *     Serial.println("failed to send CAN data");
     *   }
     * }
     * ```
     */
    int sendChar(uint32_t id, char data, uint32_t waitTime = MAX_TRANSMIT);

    /**
     * @brief charをCANで送る関数 beginでidを指定していなければfailする
     *
     * @param[in] data char型の送信するデータ
     *
     * @retval 0 success
     * @retval 1 idが指定されていない begin内でidを指定することが必要
     * @retval 2 不正なid idは11bitまで
     * @retval 3 データに誤りがある
     * @retval 4 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
     * @retval 5 twaiドライバが動作していない
     * @retval 6 unknown error
     *
     * ```cpp
     * // example
     * if (Serial.available()) {
     *   char cmd = Serial.read()
     *   Serial.println(cmd);
     *   if (CAN.sendChar(cmd)) {
     *     Serial.println("failed to send CAN data");
     *   }
     * }
     * ```
     */
    int sendChar(char data, uint32_t waitTime = MAX_TRANSMIT);

    /**
     * @brief 8文字までの文字を送信できる関数
     * charの終端にnull文字がある文字列を送信することができる関数
     * 利用するには相手側がreadLine関数かreadWithDetail関数を採用している必要がある
     * このため、begin関数内で渡される can_setting_t の multiData_send がfalseである場合は送信されない
     *
     * @warning 最後にnull文字がないとespがパニックするため注意!!!
     *
     * @param[in] id CANの送信するid
     * @param[in] data 送信したいdata
     *
     * @retval 0 success
     * @retval 2 charの配列が8文字以上あった CANは8文字以下しか送信できない
     * @retval 3 不正なid idは11bitまで
     * @retval 4 データに誤りがある
     * @retval 5 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
     * @retval 6 twaiドライバが動作していない
     * @retval 7 unknown error
     * @retval -1 multiData_send がfalseなため、複数データを送信できない
     *
     * ```cpp
     * // example
     * if (Serial.available())
     * {
     *   char data = Serial.peek(); // どの文字が入力されたかを確認(取り出さない)
     *   if (data == '\n' || data == '\r')
     *   {
     *     Serial.read(); // 改行文字だったら取り出して捨てる
     *     return;
     *   }
     *   char cmd[9] = {}; // 0の配列で初期化 null終端のために必要
     *   for (int i = 0; i < 8; i++)
     *   {
     *     while (!Serial.available())
     *       ;
     *     char read = Serial.read();
     *     if (read == '\n' || read == '\r')
     *       break; // 改行文字だったら終了
     *     cmd[i] = read;
     *     Serial.print(read);
     *   }
     *   Serial.println();
     *   if (CAN.sendLine(10, cmd)) // id 10で送信
     *   {
     *     Serial.println("failed to send CAN data");
     *   }
     * }
     * ```
     */
    int sendLine(uint32_t id, char *data, uint32_t waitTime = MAX_TRANSMIT);

    /**
     * @brief 8文字までの文字を送信できる関数
     * charの終端にnull文字がある文字列を送信することができる関数
     * 利用するには相手側がreadLine関数かreadWithDetail関数を採用している必要がある
     * charの文字列を送信する以外の用途であれば sendData 関数のほうが良い
     *
     * @warning 最後にnull文字がないとespがパニックするため注意!!!
     *
     * @param[in] data 送信したいdata
     *
     * @retval 0 success
     * @retval 1 idが指定されていない begin内でidを指定することが必要
     * @retval 2 charの配列が8文字以上あった CANは8文字以下しか送信できない
     * @retval 3 不正なid idは11bitまで
     * @retval 4 データに誤りがある
     * @retval 5 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
     * @retval 6 twaiドライバが動作していない
     * @retval 7 unknown error
     * @retval -1 multiData_send がfalseなため、複数データを送信できない
     *
     * ```cpp
     * // example
     *   if (Serial.available())
     * {
     *   char data = Serial.peek(); // どの文字が入力されたかを確認(取り出さない)
     *   if (data == '\n' || data == '\r')
     *   {
     *     Serial.read(); // 改行文字だったら取り出して捨てる
     *     return;
     *   }
     *   char cmd[9] = {}; // 0の配列で初期化 null終端のために必要
     *   for (int i = 0; i < 8; i++)
     *   {
     *     while (!Serial.available())
     *       ;
     *     char read = Serial.read();
     *     if (read == '\n' || read == '\r')
     *       break; // 改行文字だったら終了
     *     cmd[i] = read;
     *     Serial.print(read);
     *   }
     *   Serial.println();
     *   if (CAN.sendLine(cmd))
     *   {
     *     Serial.println("failed to send CAN data");
     *   }
     * }
     * ```
     */
    int sendLine(char *data, uint32_t waitTime = MAX_TRANSMIT);

    /**
     * @brief 8文字までの文字を送信できる関数
     * uint8_t型の8個までの配列を送信できる
     * 利用するには相手側がreadLine関数かreadWithDetail関数を採用している必要がある
     * charの文字列を送信する以外の用途であれば sendData 関数のほうが良い
     * @warning numに配列の正確な個数を入れないと正常に送信されなかったりESPがパニックしたりするため注意
     *
     * @param[in] id CANの送信するid
     * @param[in] data 送信したいdata
     * @param[in] num 送信したいdataの個数
     *
     * @retval 0 success
     * @retval 2 charの配列が8文字以上あった CANは8文字以下しか送信できない
     * @retval 3 不正なid idは11bitまで
     * @retval 4 データに誤りがある
     * @retval 5 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
     * @retval 6 twaiドライバが動作していない
     * @retval 7 unknown error
     * @retval -1 multiData_send がfalseなため、複数データを送信できない
     *
     * ```cpp
     * uint8_t Data[6] = {'c', 'r', 'e', 'a', 't', 'e'};
     * if (CAN.sendData(10, Data, 6)){ // 正しい文字数を入力!!!(ここでは6)
     *   Serial.println("failed to send CAN data");
     * }
     * ```
     */
    int sendData(uint32_t id, uint8_t *data, int num, uint32_t waitTime = MAX_TRANSMIT);

    /**
     * @brief 8文字までの文字を送信できる関数
     * uint8_t型の8個までの配列を送信できる
     * 利用するには相手側がreadLine関数かreadWithDetail関数を採用している必要がある
     * charの文字列を送信する以外の用途であれば sendData 関数のほうが良い
     * @warning numに配列の正確な個数を入れないと正常に送信されなかったりESPがパニックしたりするため注意
     *
     * @param[in] data 送信したいdata
     * @param[in] num 送信したいdataの個数
     *
     * @retval 0 success
     * @retval 1 idが指定されていない begin内でidを指定することが必要
     * @retval 2 charの配列が8文字以上あった CANは8文字以下しか送信できない
     * @retval 3 不正なid idは11bitまで
     * @retval 4 データに誤りがある
     * @retval 5 txキューがいっぱい 送信間隔が早すぎるかそもそも送信ができてないか
     * @retval 6 twaiドライバが動作していない
     * @retval 7 unknown error
     * @retval -1 multiData_send がfalseなため、複数データを送信できない
     *
     * ```cpp
     * uint8_t Data[6] = {'c', 'r', 'e', 'a', 't', 'e'};
     * if (CAN.sendData(10, Data, 6)){ // 正しい文字数を入力!!!(ここでは6)
     *   Serial.println("failed to send CAN data");
     * }
     * ```
     */
    int sendData(uint8_t *data, int num, uint32_t waitTime = MAX_TRANSMIT);
};

#endif