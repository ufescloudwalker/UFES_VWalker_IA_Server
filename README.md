# UFES_VWalker_IA_Server
This repository contains the source code for the control server for a smart walker. The system uses an AI agent, built with LangChain and LangGraph and powered by a local language model via Ollama, to provide an intuitive and assistive navigation interface. The server acts as a bridge between user commands and the ROS 2 robotics ecosystem, and also provides audio feedback via a Text-to-Speech (TTS) service.

##Architecture
The system is designed around a Flask API that exposes endpoints for user interaction. The core logic is encapsulated in an AI agent that interprets the user’s intent and executes the appropriate tools to control the walker.

The interaction flow is as follows:

1. User Input: The user sends a text command (e.g., “Take me to the kitchen” or “I want to walk freely”).
2. API Server (vwalker_server_API.py): Receives the HTTP request.
3. AI Agent (ollama_functions.py): The agent, guided by a strict operational prompt (agent_prompt.py), processes the input to classify the intent:

. Assisted Navigation: The user wants to go to a specific location.
. Free Navigation: The user wants to control the walker manually.
. Training Mode: The user wants to learn how to use the walker.

5. Tool Execution (agent_tools.py): Based on the intent, the agent calls the appropriate tools, which are ROS 2 clients. These tools communicate with the walker nodes to:

.Enable/disable control modes.
.Request Nav2 to calculate a route.
.Load a predefined training path.

6. User Response: The agent generates a text response.
7. Speech Synthesis (tts.py): The server uses the Suno Bark model to convert the agent's response into audio, providing clear and accessible feedback.
8. Frontend Action: The server can send a trigger_action (e.g. OPEN_WINDOW) to the frontend, allowing the user interface to react dynamically (e.g. opening a navigation screen).

##Features
1. Natural Language Processing: Understands user commands in natural language for easy interaction.
2. Three Operation Modes:
. Assisted Navigation: Guides the user to a specific destination by generating a path through Nav2.
. Free Navigation: Allows the user to control the walker autonomously.
. Training Mode: Loads a predefined path (lemniscate_medium_4x4.csv) so the user can practice using the walker.
3. ROS 2 Integration: Interacts directly with ROS 2 services and actions for real-time robot control.
4. Voice Feedback: Uses a high-quality TTS model to provide audible instructions and responses.
5. Robust API: Flask-based API for easy integration with different types of user interfaces (web, mobile, etc.).

##Pré-requisites

Before you begin, make sure you have the following installed:

. Python 3.9 or higher.
. ROS 2 Humble Hawksbill: The ROS 2 environment must be installed and configured.
. Ollama: To run the language model locally. Installation instructions.
. LLM Model: The code is configured to use qwen2.5:14b. Download it with:
