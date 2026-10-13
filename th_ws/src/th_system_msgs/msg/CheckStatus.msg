std_msgs/Header header

string item
string result        # "OK"/"WARN"/"NG"/"UNKNOWN"
string detail
string next_screen
string basis_json    # 判定の根拠。JSON 配列 [{"key","value","threshold","unit","cmp","ok"}, ...]。
                     # 最終判定（OK/WARN/NG）のときだけ載せ、無いときは空文字。ラベルは WebUI の i18n が持つ。
