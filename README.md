# VENDOR DEP
`https://raw.githubusercontent.com/FRC-6390/FRC-Athena/refs/heads/main/FRC6390-Athena.json`

# Publishing
```bash
export $(cat .env | xargs)
 ./gradlew publish -PpublishMode=all -PfrcYear=2025 #Publish local to wpilib folder and online
 ```

 ```bash
 ./gradlew publish -PpublishMode=local -PfrcYear=2025 -Pversion=2025.9.9 #Publish local to wpilib folder
 ```

 ```bash
export $(cat .env | xargs)
 ./gradlew publish -PpublishMode=online -PfrcYear=2025 #Publish to online
 ```