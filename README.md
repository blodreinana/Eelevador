# 🛗 Challenge - Dispositivo de Diagnóstico Inteligente para Elevadores

Este repositório contém o projeto desenvolvido para o **Challenge 2025 da FIAP**, em parceria com a empresa **Otis**, com o objetivo de criar uma solução inovadora para instalação, manutenção e análise de riscos em elevadores.

---

## 🎯 Propósito do Projeto

Desenvolver um **dispositivo plug and play inteligente** que possa ser facilmente conectado a um elevador recém-instalado ou em operação, oferecendo:

- Diagnóstico rápido de falhas
- Detecção de riscos operacionais
- Monitoramento de vibração, temperatura e alinhamento
- Mapeamento do poço do elevador
- Armazenamento dos dados na nuvem
- Leitura de dados por sistema dedicado e interface para análise

O sistema visa otimizar o trabalho dos técnicos e aumentar a segurança do processo de instalação e manutenção, garantindo maior precisão, agilidade e confiabilidade.

---

## 🧠 Funcionalidades

- 🔍 **Leitura em tempo real** dos sensores conectados ao ESP32
- 📈 **Análise de deslocamento vertical** com sensor LiDAR
- 🧭 **Medição de vibração e aceleração** com acelerômetro
- 🌡️ **Monitoramento ambiental** com sensores opcionais (como DHT22)
- ☁️ **Envio dos dados para a nuvem** para posterior visualização e análise
- 🖥️ **Leitura dos dados via sistema dedicado**, com possibilidade futura de dashboard web
- 💾 Backup local dos dados via cartão SD (versão futura)
- 🚨 Sistema de alertas baseado em padrões de anomalia

---

## 🧰 Componentes e Tecnologias

- **ESP32** – microcontrolador principal
- **Sensor LiDAR** – mapeamento do poço do elevador
- **Acelerômetro** – detecção de vibração e inclinação
- **Sensor DHT22** – monitoramento de temperatura e umidade (opcional)
- **Firebase / Google Sheets / WebApp (futuro)** – integração com nuvem
- **Cartão SD (opcional)** – para armazenar dados offline
- **Wokwi** – simulação inicial
- **Arduino IDE** – programação embarcada

---

## 📊 Estado do Projeto

- [x] Coleta de dados com sensores via ESP32
- [x] Visualização dos dados no monitor serial
- [ ] Envio dos dados para a nuvem (Firebase ou similar)
- [ ] Desenvolvimento do sistema de leitura dos dados na interface web/app
- [ ] Armazenamento local com cartão SD (backup)
- [ ] Sistema de alertas visuais e lógicos

> ✔️ Primeira versão funcional finalizada para apresentação.  
> 🔄 Melhorias futuras previstas para integrar a nuvem e criar um sistema visual de leitura.

---

## 👩‍💻 Desenvolvido por

**Allana Helena Campos Albino**  
**Tiago Faria de Almeida Ferreira**
**Erick Lima Barbosa**
**Mateus Hipolito Bustamante**
**Pedro Lopes Ferreira**
**Luiz Henrique de Almeida**
FIAP – Engenharia Mecatrônica (1º semestre)

Desenvolvido com dedicação, noites em claro, café e amor por tecnologia 💙

---

## 📎 Sobre o Challenge

O **Challenge FIAP** é uma iniciativa interdisciplinar que promove inovação e resolução de problemas reais por meio de projetos práticos em parceria com empresas. Nesta edição, o desafio foi proposto pela **Otis**, líder mundial em elevadores.

---

## 💬 Contato

Para dúvidas, sugestões ou colaborações:

- GitHub: [@blodreinana](https://github.com/blodreinana)
- LinkedIn: [Tiago Ferreira](https://www.linkedin.com/in/tiago-ferreira-65886134b) *(mentor do projeto)*

---

🛗 *Tecnologia em movimento para um futuro mais seguro.*
