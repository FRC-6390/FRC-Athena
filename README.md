# VENDOR DEP
`https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/FRC6390-Athena.json`

# Publishing
```bash
export $(cat .env | xargs)
 ```

 ```bash
 ./gradlew publish -PpublishMode=local -PfrcYear=2025 -Pversion=2025.9.11 #Publish local to wpilib folder
 ```

 ```bash
export $(cat .env | xargs)
 ./gradlew publish -PpublishMode=online -PfrcYear=2025 -Pversion=2025.9.11 #Publish to online
 ```