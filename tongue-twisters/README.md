# 繞口令每日寄信

本資料夾包含 20 個繞口令(每個 15–20 字),以及一個每天自動寄信給
`myftptoyman@gmail.com` 的 GitHub Actions 工作流。

## 檔案

- `tongue_twisters.md` — 20 個繞口令(含字數表格)
- `tongue_twisters.txt` — 純文字版,工作流寄信時使用的內容
- `../.github/workflows/daily-tongue-twisters.yml` — 每日寄信工作流

## 啟用每日寄信(只需設定一次)

由於寄信需要一個寄件帳號,請在 GitHub repo 設定兩個 secret:

1. 開啟 repo 的 **Settings → Secrets and variables → Actions → New repository secret**
2. 新增:
   - `MAIL_USERNAME`:寄件用的 Gmail,例如 `yourname@gmail.com`
   - `MAIL_PASSWORD`:該 Gmail 的**應用程式密碼 (App Password)**
     - 建立方式:Google 帳號 → 安全性 → 開啟兩步驟驗證 → 應用程式密碼
     - 注意:不是平常登入的密碼

設定完成後:

- 工作流每天 **01:07 UTC(約台灣時間早上 09:07)** 自動寄出。
- 也可到 repo 的 **Actions → Daily Tongue Twisters Email → Run workflow**
  手動觸發一次,測試是否能成功收到信。

## 調整寄信時間

編輯 `.github/workflows/daily-tongue-twisters.yml` 裡的 `cron`(UTC 時間)。
例如改成台灣時間晚上 8 點(20:00,即 12:00 UTC):

```yaml
    - cron: "0 12 * * *"
```
