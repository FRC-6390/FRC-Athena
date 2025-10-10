# VENDOR DEP
`https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/FRC6390-Athena.json`

# Publishing
```bash
export $(cat .env | xargs)
 ```

```bash
 ./gradlew publish -PpublishMode=local # version/year auto-resolve
 ```

 ```bash
export $(cat .env | xargs)
 ./gradlew publish -PpublishMode=online # version/year auto-resolve
 ```

To publish with a specific version, append `-Pversion=<major.minor.patch>` to override the automatic increment. To pin the FRC season manually, append `-PfrcYear=<season>`.
