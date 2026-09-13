---
name: pr-review
description: PR のレビューコメントに対応する。未解決スレッドを読み、修正してコミットし、返信と Resolve を行う。「レビューコメント対応」「レビュー見て」「PR のコメントを直して」などと言った時に使用する。
allowed-tools: Bash(gh pr view:*), Bash(gh api:*), Bash(git:*), Read, Edit, Grep, Glob
---

# PR Review Comments

PR の未解決レビュースレッドを 1 つずつ片付ける。

## 1. 未解決スレッドを集める

PR 番号は引数で受け取る。無ければ `gh pr view --json number -q .number`。

```shell
gh api graphql --paginate -f query='
query($owner:String!,$repo:String!,$num:Int!,$endCursor:String){
  repository(owner:$owner,name:$repo){ pullRequest(number:$num){
    reviewThreads(first:100, after:$endCursor){
      nodes{
        id isResolved isOutdated path line
        comments(first:100){ nodes{ databaseId author{login} body } } }
      pageInfo{ hasNextPage endCursor } } } }
}' -F owner=ForteFibre -F repo=fibril_zephyr -F num=<N> \
  --jq '.data.repository.pullRequest.reviewThreads.nodes[] | select(.isResolved | not)'
```

**`--paginate` を外さない。** レビュアーが bot だとスレッドは簡単に 100 を超える。
黙って先頭 100 件だけを見て、残りを対応済みと誤認する。

`--paginate` が使う変数名は `$endCursor` で固定されている。
`pageInfo` を持つ一番外側の接続だけを辿るので、`comments` の側には `pageInfo` を置かない。
返信先に使う先頭コメントは必ず最初のページにあるため、これで困らない。

`--jq` はページごとに適用される。出力は JSON の配列ではなく 1 スレッド 1 行になる。

`isOutdated` のスレッドも読む。行がずれただけで指摘が生きていることがある。

## 2. 方針を出して承認を取る

スレッドごとに「直す」か「理由を添えて見送る」かを一覧にし、**着手前にユーザーの承認を 1 回取る**。
返信と Resolve は PR に出る外向きの操作なので、承認なしに実行しない。

## 3. 直してコミットする

修正は論理単位でコミットを分ける（`CLAUDE.md` の「コミット」に従う）。
コードを変えたら対応する文書も同じ PR で直す（「ドキュメント更新責務」の表）。
コミットし終えたら push する。

## 4. 返信して Resolve する

返信先はスレッド先頭コメントの `databaseId`（返信への返信は作れない）。
本文には何をどう直したかと修正コミットの SHA を書く。見送るならその理由を書く。

```shell
gh api repos/ForteFibre/fibril_zephyr/pulls/<N>/comments/<databaseId>/replies -f body='...'

gh api graphql -f query='mutation($id:ID!){
  resolveReviewThread(input:{threadId:$id}){ thread{ isResolved } } }' -F id=<threadId>
```

見送ったスレッドは Resolve しない。返信だけ残してユーザーの判断に委ねる。
