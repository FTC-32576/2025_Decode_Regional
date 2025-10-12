
**🌎 Idiomas disponíveis:**
[![English](https://img.shields.io/badge/lang-en-red.svg)](readme.md)
[![Português](https://img.shields.io/badge/lang-pt--br-green.svg)](readme.pt.md)

# Java Is All Might

Este repositório contém o código oficial da **equipe FTC All Might #32576** para a temporada atual da FIRST Tech Challenge (FTC).
Nosso projeto segue o **NGC — Next Generation Codebase**, um modelo de organização desenvolvido pela própria equipe para manter o código limpo, escalável e sustentável entre temporadas.

## 🧩 O que é o NGC (Next Generation Codebase)

O **NGC** é o padrão de arquitetura de código adotado pela All Might.
Seu objetivo é separar o código **genérico (reutilizável)** do código **específico da temporada**, permitindo que a base seja reaproveitada com mínima refatoração.

Essa separação garante:

* 🔁 Reutilização entre temporadas (ex: classes de PID, IMU, Limelight);
* 🧱 Modularidade — cada parte do robô é independente;
* ⚙️ Manutenção mais fácil e segura;
* 🚀 Padronização entre desenvolvedores e subequipes.

---

## ⚙️ Organização do Código

| Categoria             | Robot Code                                                           | Library Code (TeamLib / Java_Is_AllMight)                   |
| --------------------- | -------------------------------------------------------------------- | ----------------------------------------------------------- |
| **Função**            | Código específico da temporada atual | Código genérico, independente da temporada                  |
| **Escopo**            | `ftc.team.allmight.plusultra.teamcode`                                       | `ftc.team.Java_Is_AllMight`                                         |
| **Responsabilidade**  | Controlar o robô em campo, definir estratégias e rotinas autônomas   | Fornecer ferramentas, algoritmos e abstrações reutilizáveis |
| **Qualidade** | Deve funcionar e estar bem testado                                   | 	Estilo de código consistente, legibilidade e documentação completa. Deve ser robusto e reutilizável.             |
| **Ciclo de vida**     | Dura apenas a temporada atual                                        | Dura várias temporadas                                      |
> 💡 **NGC Insight:** A separação Robot / Library garante que mudanças de estratégia ou hardware na temporada não impactem a lógica genérica de controle e sensores. Isso torna a base mais sustentável e escalável para futuras temporadas.

---

## 🗂️ Estrutura do Projeto

```text
TeamCode/
│
├── Java/ftc/team/
│
├── Java_Is_AllMight/                # TeamLib - Library Code (reutilizável)
│   ├── Control/                     # PID, controle de movimento, navegação
│   ├── Sensors/                     # IMUHelper, LimelightHelper e sensores
│   ├── Logging/                     # Logs e diagnóstico do robô
│
├── allmight/plusultra/teamcode/     # Robot Code (temporada atual)
│   ├── TeleOp/                      # Modos manuais
│   ├── Auto/                        # Modos autônomos
│   └── Subsystems/                  # Sub-sistemas físicos (Drive, Shooter, Intake)
│
└── build.gradle                     # Configuração Android / FTC SDK
```


---

## 🧠 TeamLib - Java_Is_AllMight

A **TeamLib** é o coração da base de código.
Ela contém tudo que é **independente de robô ou temporada**, formando uma biblioteca interna reutilizável.

### Estrutura:

* **Control/**

    * PIDConfig, PIDController;
* **Sensors/**

    * `IMUHelper`, `LimelightHelper`;
* **Logging/**

    * Classes para coleta e exibição de dados, diagnósticos e debug;


> 📘 **Missão da TeamLib:**
> “Write once, use forever.” — Tudo o que for genérico deve nascer na TeamLib para ser herdado por futuras temporadas.

---

## ⚙️ Robot Code

O **Robot Code** é onde o robô da temporada é realmente programado.
Aqui entram as estratégias, rotinas automáticas e controles manuais.
```
          ┌────────────────────┐
          │       OpMode       │
          │  (TeleOp / Auto)   │
          └─────────┬──────────┘
                    │
        ┌───────────▼───────────┐
        │   Robot Subsystems    │
        │ (Drive, Shooter, etc) │
        └───────────┬───────────┘
                    │
        ┌───────────▼───────────────┐
        │         TeamLib           │
        │ Control | Sensors | Utils │
        └───────────┬───────────────┘
                    │
        ┌───────────▼────────────┐
        │      HardwareMap       │
        │ Motores, Sensores, IMU │
        └────────────────────────┘
```

* **OpMode:** ponto de entrada da operação (manual ou autônoma).
* **Subsystems:** recebem comandos e controlam motores e sensores.
* **TeamLib:** fornece cálculos, helpers e lógica de controle.
* **HardwareMap:** interface direta com o hardware do robô.
> 🧩 Cada subsystem utiliza classes da TeamLib para realizar tarefas complexas (PID, Limelight, IMU), mantendo o código limpo e focado na lógica da partida.
---

## 🔹 Continuous Integration (CI) & Qualidade

A equipe All Might utiliza um pipeline de **CI/CD (Continuous Integration / Continuous Deployment)** para manter a estabilidade e qualidade do código.

### 🔄 O que o pipeline faz:

* **✅ Validação automática:**
  Cada *push* ou *pull request* executa uma checagem automatizada no GitHub Actions, validando:

    * Estrutura de pacotes;
    * Compilação do projeto FTC (gradle build);
    * Conformidade de estilo e documentação.

* **🧪 Testes de unidade e integração:**
  Rotinas da TeamLib (como PID, IMUHelper e LimelightHelper) possuem testes simulados para validar comportamento matemático e estrutural.

* **📦 Deploy automatizado (opcional):**
  Quando uma versão estável é marcada como *release*, o pipeline gera automaticamente um build `.apk` assinado e pronto para ser instalado no **Robot Controller (RC)**.

> 💡 Essa abordagem garante que **nenhuma modificação chegue ao robô sem passar por verificação técnica**, aumentando a confiabilidade e reduzindo erros em campo.

---

## 🧱 Filosofia de Desenvolvimento

> “Our codebase is split into two distinct parts: **robot code** and **library code**.
> The robot code contains everything that you would expect from a command-based project, while the library contains code that should be reusable for multiple seasons.”

Essa filosofia garante:

* 🔧 Facilidade de manutenção;
* ♻️ Reaproveitamento entre temporadas;
* 📚 Documentação clara e universal;
* 🚀 Evolução contínua da base técnica da All Might.

---


## 👑 Créditos

Desenvolvido por **Equipe FTC All Might #32576**

Liderança de Software: [**Nobre**](https://github.com/meuNobre)

Baseado na filosofia **Next Generation Codebase (NGC)**

