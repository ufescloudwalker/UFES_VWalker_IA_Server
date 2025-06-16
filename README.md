# UFES_VWalker_IA_Server

Este repositório contém o código-fonte do servidor de controle para um andador inteligente. O sistema utiliza um agente de IA, construído com **LangChain** e **LangGraph** e alimentado por um modelo de linguagem local via **Ollama**, para oferecer uma interface de navegação intuitiva e assistiva. O servidor atua como uma ponte entre os comandos do usuário e o ecossistema robótico **ROS 2**, além de fornecer feedback em áudio por meio de um serviço de **Text-to-Speech (TTS)**.

---

## Arquitetura

O sistema é estruturado em torno de uma API Flask que expõe endpoints para interação com o usuário. A lógica central está encapsulada em um agente de IA que interpreta a intenção do usuário e executa as ferramentas apropriadas para controlar o andador.

### Fluxo de Interação:

1. **Comando do Usuário:** O usuário envia um comando de texto (ex: “Leve-me até a cozinha” ou “Quero andar livremente”).
2. **Servidor API (`vwalker_server_API.py`):** Recebe a requisição HTTP.
3. **Agente de IA (`ollama_functions.py`):** O agente, guiado por um prompt operacional rigoroso (`agent_prompt.py`), processa o input para classificar a intenção:
   - Navegação Assistida
   - Navegação Livre
   - Modo de Treinamento
4. **Execução das Ferramentas (`agent_tools.py`):** De acordo com a intenção, o agente chama as ferramentas adequadas (clientes ROS 2), que se comunicam com os nós do andador para:
   - Habilitar/desabilitar modos de controle.
   - Requisitar ao Nav2 o cálculo de uma rota.
   - Carregar um caminho de treinamento pré-definido.
5. **Resposta ao Usuário:** O agente gera uma resposta em texto.
6. **Síntese de Voz (`tts.py`):** O servidor usa o modelo **Suno Bark** para converter a resposta em áudio, fornecendo feedback acessível.
7. **Ação no Frontend:** O servidor pode enviar um `trigger_action` (ex: `OPEN_WINDOW`) para que a interface reaja dinamicamente (ex: abrir a tela de navegação).

---

##  Funcionalidades

- **Processamento de Linguagem Natural:** Compreende comandos em linguagem natural para uma interação facilitada.
- **Três Modos de Operação:**
  - **Navegação Assistida:** Guia o usuário até um destino específico utilizando o Nav2.
  - **Navegação Livre:** Permite que o usuário controle o andador manualmente.
  - **Modo de Treinamento:** Carrega um trajeto de treino pré-definido (`lemniscate_medium_4x4.csv`) para que o usuário pratique.
- **Integração com ROS 2:** Comunicação direta com serviços e ações do ROS 2 para controle robótico em tempo real.
- **Feedback por Voz:** Utiliza um modelo TTS de alta qualidade para fornecer instruções e respostas audíveis.
- **API Robusta:** Baseada em Flask, facilita a integração com diferentes interfaces de usuário (web, mobile, etc).

---

##  Pré-requisitos

Antes de começar, certifique-se de que os seguintes itens estão instalados:

- Python **3.9** ou superior  
- **ROS 2 Humble Hawksbill**: Ambiente ROS 2 instalado e configurado  
- **Ollama**: Para rodar o modelo de linguagem local ([instruções de instalação](https://ollama.com))  
- **Modelo LLM**: O código está configurado para utilizar o `qwen2.5:14b`. Faça o download com:
  ```bash
  ollama run qwen2.5:14b
