# nav_world map golden の FMA 融合修正

## 目的

`ros2_autonomous_nav` の `MapGenerator.MatchesGoldenPgmByteForByte` が `main` で落ちている。
原因を特定し、**golden（`maps/nav_world.pgm`）を書き換えずに** C++ 生成器を
Python 版と同じ丸めへ戻す。

## 確定した原因（一次情報 / 実測）

- 失敗は row 140 col 88（world x=-1.575, y=-1.025）で、golden は free (254)、
  C++ 生成器は occupied (0)。差分は **136 セル**で、内部壁 `wall_i1`
  （center -1.5, size 0.15）の両端（ちょうど半セル）に集中している。
- 削除前の Python 実装（`2439065^:scripts/generate_map.py`）を numpy で実行すると、
  出力 PGM は **golden とバイト一致**（`cmp` exit 0）。境界判定は同じ `<=`。
- 同一セルの `lx` は、Python / 分離丸めでは `-0.07500000000000018`（free）、
  GCC の `-O3`（既定 `-ffp-contract=fast`）では `-0.07499999999999973`（occupied）。
  `origin + (px+0.5)*resolution` が FMA に融合され、丸めが 1 回になるため。
- `-O3 -ffp-contract=off` で生成器を再コンパイルすると差分 **0**（golden と完全一致）。
  既定の `-O3` では 136。測定点は上記のセルと全 240×240 の比較。

## 修正

- `map_generator` ターゲットだけに `-ffp-contract=off` を付ける（GNU/Clang）。
  Python 版の「乗算と加算を別々に丸める」契約をビルドで固定する。
- `WallBoundaryMatchesPythonRounding` を追加し、壁の両端が free、内部が occupied で
  あることをピン留めする。golden テストは既存のまま使う。
- golden / SDF / Python 互換の契約は変更しない。

## 検証

1. 追加テストと golden テストが修正前 RED（両方失敗）、修正後 GREEN。
2. `colcon test --packages-select ros2_autonomous_nav` が 66 tests / 0 failures。
3. ワークスペース全体の `colcon test` が 0 failures（この失敗が最後の既知の赤）。
4. `git diff --check`。

## 非ゴール

- golden の再生成（地図の意味が変わるため却下）。
- map の解像度・ジオメトリ・Nav2 パラメータの変更。
- Python 実装の復活。

## コミット

1. `docs(superpowers): add the nav map golden fp-contract plan`
2. `fix(nav): keep the map golden byte-identical under FMA contraction`
