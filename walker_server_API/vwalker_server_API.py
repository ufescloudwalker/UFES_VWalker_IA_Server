from flask import Flask, request, jsonify
from flask_cors import CORS
from flask import Response
from utils.ollama_functions import agentCreator 
import io
import soundfile as sf
import torch
import requests
import json
import logging
import sys
import os

# Adicionar 'utils' ao path para importação
# Se 'utils' está no mesmo diretório que vwalker_server_API.py:
# Esta linha já está correta para a estrutura de pastas inferida
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

OLLAMA_BASE_URL = "http://localhost:11434" 
MODEL_NAME = "qwen2.5:14b"

agent_executor = None
try:
    if agentCreator: 
        agent_executor = agentCreator() 
        logger.info(f"Agente customizado inicializado com o modelo {MODEL_NAME} (configurado em ollama_functions.py).")
    else:
        logger.warning("agentCreator não pôde ser importado. Endpoint do agente não funcionará.")

except ImportError as e:
    logger.error(f"Falha ao importar 'agentCreator' de 'utils.ollama_functions': {e}. Verifique o PYTHONPATH e os arquivos.")
    agentCreator = None 
except Exception as e:
    logger.error(f"Erro durante a inicialização do agente: {e}", exc_info=True)
    agent_executor = None


try:
    from utils.tts import TextToSpeechService 
    tts_service_on_server = TextToSpeechService()
    logger.info("Serviço de Text-to-Speech (Bark) inicializado no servidor.")
except Exception as e:
    tts_service_on_server = None
    logger.error(f"Falha ao inicializar o serviço TTS no servidor: {e}", exc_info=True)



from langchain_core.messages import HumanMessage

class OllamaClient:
    def __init__(self, base_url, model_name):
        self.base_url = base_url
        self.model_name = model_name
    
    def generate(self, prompt, stream=False, temperature=0.7, max_tokens=2048):
        url = f"{self.base_url}/api/generate"
        payload = {"model": self.model_name, "prompt": prompt, "stream": stream, "options": {"temperature": temperature, "num_predict": max_tokens}}
        try:
            response = requests.post(url, json=payload, timeout=120); response.raise_for_status()
            return response.json() if not stream else response
        except requests.exceptions.RequestException as e: logger.error(f"OllamaClient.generate error: {e}"); raise
    
    def chat(self, messages, stream=False, temperature=0.7, max_tokens=2048):
        url = f"{self.base_url}/api/chat"
        payload = {"model": self.model_name, "messages": messages, "stream": stream, "options": {"temperature": temperature, "num_predict": max_tokens}}
        try:
            response = requests.post(url, json=payload, timeout=120); response.raise_for_status()
            return response.json() if not stream else response
        except requests.exceptions.RequestException as e: logger.error(f"OllamaClient.chat error: {e}"); raise

    def check_model_status(self):
        try:
            url = f"{self.base_url}/api/tags"; response = requests.get(url, timeout=10); response.raise_for_status()
            models = response.json().get('models', []); model_names = [model['name'] for model in models]
            return self.model_name in model_names
        except requests.exceptions.RequestException as e: logger.error(f"OllamaClient.check_model_status error: {e}"); return False

ollama_direct_client = OllamaClient(OLLAMA_BASE_URL, MODEL_NAME)


@app.route('/health', methods=['GET'])
def health_check():
    model_available = ollama_direct_client.check_model_status()
    return jsonify({
        'status': 'healthy' if model_available else 'model_unavailable',
        'model_for_direct_access': MODEL_NAME,
        'model_available': model_available,
        'agent_initialized_on_server': agent_executor is not None
    })

@app.route('/generate', methods=['POST'])
def generate_route():
    try: data = request.get_json(); result = ollama_direct_client.generate(prompt=data['prompt']); return jsonify(result)
    except Exception as e: return jsonify({'error': str(e)}), 500


@app.route('/chat', methods=['POST'])
def chat_route():
    try: data = request.get_json(); result = ollama_direct_client.chat(messages=data['messages']); return jsonify(result)
    except Exception as e: return jsonify({'error': str(e)}), 500


@app.route('/agent/invoke', methods=['POST'])
def agent_invoke_route():
    if not agent_executor:
        return jsonify({'error': 'Agente não está inicializado no servidor.'}), 503

    try:
        data = request.get_json()
        user_query = data.get('query') or data.get('input_text')
        
        if not user_query:
            return jsonify({'error': 'Os campos "query" ou "input_text" são obrigatórios.'}), 400

        invocation_payload = {"messages": [HumanMessage(content=user_query)]}
        logger.info(f"Invocando agente do servidor com payload: {invocation_payload}")
        
        #Process of extracting information from the main agent's response
        agent_response_state = agent_executor.invoke(invocation_payload)
        # Extract the final textual response and the last AI message for fallback
        final_response_text = "O agente não forneceu uma resposta no formato esperado."
        last_ai_message_content = None 

        if agent_response_state and "messages" in agent_response_state and agent_response_state["messages"]:
            # Get the last general message for reference
            # And the last AIMessage specifically for the final text, if no relevant tool provides one.
            for msg in reversed(agent_response_state["messages"]):
                if msg.type == 'ai' and hasattr(msg, 'content'): 
                    last_ai_message_content = msg.content
                    if msg.tool_calls: 
                        pass 
                    else: 
                        final_response_text = msg.content if msg.content else final_response_text
                    break 
            if last_ai_message_content is None and agent_response_state["messages"]: 
                final_response_text = agent_response_state["messages"][-1].content if hasattr(agent_response_state["messages"][-1], 'content') else final_response_text

        logger.info(f"Texto final preliminar do agente (última AIMessage ou fallback): '{final_response_text}'")

        nomes_ferramentas_gatilho = ["create_path_for_navigation", "start_training_mode"] 
        ferramenta_gatilho_usada = False
        nome_ferramenta_especifica_usada = None 
        argumentos_para_trigger = {} 
        trigger_action_payload = None

        # Iterar por TODAS as mensagens para encontrar a ÚLTIMA execução BEM-SUCEDIDA
        # de uma ferramenta gatilho, especialmente 'create_path_for_navigation'.
        # Isso é mais robusto do que depender apenas de 'intermediate_steps' ou 'last_message.tool_calls'
        # quando a estrutura de 'messages' já contém os resultados das ferramentas.

        if agent_response_state and "messages" in agent_response_state:
            # Percorrer as mensagens. Cada AIMessage com tool_calls é seguida por ToolMessages com os resultados.
            agente_sugeriu_tool_id = {} # Para mapear tool_call_id para nome da ferramenta

            for i, message in enumerate(agent_response_state["messages"]):
                if message.type == 'ai' and hasattr(message, 'tool_calls') and message.tool_calls:
                    for tc in message.tool_calls:
                        tool_call_id = tc.get('id') if isinstance(tc, dict) else getattr(tc, 'id', None)
                        tool_name = tc.get('name') if isinstance(tc, dict) else getattr(tc, 'name', None)
                        if tool_call_id and tool_name in nomes_ferramentas_gatilho:
                            agente_sugeriu_tool_id[tool_call_id] = tool_name
                            logger.info(f"LLM sugeriu ferramenta: ID='{tool_call_id}', Nome='{tool_name}'")

                elif message.type == 'tool' and hasattr(message, 'tool_call_id'):
                    tool_call_id_executado = message.tool_call_id
                    nome_ferramenta_executada = agente_sugeriu_tool_id.get(tool_call_id_executado)
                    
                    if nome_ferramenta_executada and nome_ferramenta_executada in nomes_ferramentas_gatilho:
                        logger.info(f"Processando resultado da ferramenta: Nome='{nome_ferramenta_executada}', ID Chamada='{tool_call_id_executado}'")
                        
                        # 'observation' é o conteúdo da ToolMessage
                        observation_content = message.content
                        observation_status = getattr(message, 'status', None) # Verificar se tem status='error'

                        if observation_status == 'error':
                            logger.warning(f"Ferramenta '{nome_ferramenta_executada}' executou com erro: {observation_content}")
                            continue # Pula para a próxima mensagem se esta execução falhou

                        if nome_ferramenta_executada == "create_path_for_navigation":
                            if isinstance(observation_content, str):
                                try:
                                    parsed_observation = json.loads(observation_content)
                                    if parsed_observation.get("status") == "success" and "path_points" in parsed_observation:
                                        ferramenta_gatilho_usada = True # Marcamos como usada e BEM SUCEDIDA
                                        nome_ferramenta_especifica_usada = nome_ferramenta_executada
                                        argumentos_para_trigger = parsed_observation # Usar o dict parseado
                                        
                                        # ATUALIZA final_response_text com a mensagem da ferramenta, pois é mais relevante
                                        final_response_text = parsed_observation.get("message", final_response_text)
                                        logger.info(f"SUCESSO! Ferramenta '{nome_ferramenta_executada}' processada. Params: {argumentos_para_trigger}. TTS: {final_response_text}")
                                        # Não damos break aqui, pois queremos a *última* execução bem-sucedida,
                                        # embora na prática, após um sucesso, o agente pode não tentar de novo.
                                        # Se a lógica do agente pode tentar várias vezes e você quer a primeira, ajuste.
                                        # Se quisermos a última bem-sucedida, essa lógica de sobrescrever está correta.
                                    else:
                                        logger.warning(f"'{nome_ferramenta_executada}' sucesso na observação, mas JSON não tem 'status:success' ou 'path_points': {parsed_observation}")
                                except json.JSONDecodeError as e:
                                    logger.error(f"Erro ao decodificar JSON da observação de '{nome_ferramenta_executada}': {e}. Observação: {observation_content}")
                            else:
                                logger.warning(f"Observação de '{nome_ferramenta_executada}' não é string JSON: {observation_content}")
                        
                        elif nome_ferramenta_executada == "start_training_mode":
                            # Lógica similar para 'start_training_mode' se ela retornar JSON ou dados específicos
                            ferramenta_gatilho_usada = True
                            nome_ferramenta_especifica_usada = nome_ferramenta_executada
                            try:
                                # Supondo que o training mode também retorne um JSON ou dados estruturados
                                argumentos_para_trigger = json.loads(observation_content) if isinstance(observation_content, str) else {"result": observation_content}
                                final_response_text = argumentos_para_trigger.get("message", f"Modo de treinamento {nome_ferramenta_especifica_usada} ativado.")
                                logger.info(f"SUCESSO! Ferramenta '{nome_ferramenta_executada}' processada. Params: {argumentos_para_trigger}. TTS: {final_response_text}")
                            except json.JSONDecodeError:
                                argumentos_para_trigger = {"raw_content": observation_content, "message": f"Modo de treinamento {nome_ferramenta_especifica_usada} ativado."}
                                final_response_text = argumentos_para_trigger["message"]
                                logger.info(f"SUCESSO! Ferramenta '{nome_ferramenta_executada}' processada (conteúdo não JSON). Params: {argumentos_para_trigger}. TTS: {final_response_text}")


        # Se uma ferramenta gatilho foi usada com sucesso, construir o payload da trigger_action
        if ferramenta_gatilho_usada: # Agora 'ferramenta_gatilho_usada' só é True se houve sucesso E dados corretos
            trigger_action_payload = {
                "action": "OPEN_WINDOW",
                "window_type": nome_ferramenta_especifica_usada, 
                "params": argumentos_para_trigger 
            }
            logger.info(f"Payload da trigger_action preparado: {trigger_action_payload}")
        else:
            logger.info(f"Nenhuma ferramenta gatilho relevante foi executada com sucesso ou com dados válidos.")


        # Montar a resposta final (já fizemos isso no início do código, apenas adicionamos a trigger_action)
        response_to_client = {
            'agent_response': final_response_text,
            'model_used_by_agent': MODEL_NAME 
        }
        if trigger_action_payload:
            response_to_client['trigger_action'] = trigger_action_payload

        logger.info(f"Enviando resposta para o cliente: {response_to_client}")
        return jsonify(response_to_client)
    except Exception as e:
        logger.error(f"Erro no endpoint /agent/invoke: {e}", exc_info=True)
        return jsonify({'error': str(e)}), 500

@app.route('/models', methods=['GET'])
def list_models_route():
    try: url = f"{OLLAMA_BASE_URL}/api/tags"; response = requests.get(url, timeout=10); response.raise_for_status(); return jsonify(response.json())
    except Exception as e: return jsonify({'error': str(e)}), 500

@app.route('/tts/synthesize', methods=['POST'])
def tts_synthesize_route():
    if not tts_service_on_server:
        return jsonify({'error': 'Serviço TTS não está inicializado no servidor.'}), 503

    try:
        data = request.get_json()
        text_to_synthesize = data.get('text')
        language = data.get('language', 'pt') # Idioma padrão 'pt' se não especificado

        if not text_to_synthesize:
            return jsonify({'error': 'O campo "text" é obrigatório para a síntese de fala.'}), 400

        logger.info(f"TTS Servidor: Recebido pedido para sintetizar: '{text_to_synthesize[:50]}...' em '{language}'")

        sample_rate, audio_array_numpy = tts_service_on_server.long_form_synthesize(text_to_synthesize, language)


        wav_io = io.BytesIO()
        sf.write(wav_io, audio_array_numpy, sample_rate, format='WAV')
        wav_bytes = wav_io.getvalue()
        

        if torch.cuda.is_available(): 
            torch.cuda.empty_cache()
            logger.info("TTS Servidor: Cache da GPU limpo.")

        logger.info(f"TTS Servidor: Áudio sintetizado. Enviando {len(wav_bytes)} bytes.")
        return Response(wav_bytes, mimetype='audio/wav')

    except Exception as e:
        logger.error(f"Erro no endpoint /tts/synthesize: {e}", exc_info=True)
        return jsonify({'error': f'Erro na síntese de fala no servidor: {str(e)}'}), 500



if __name__ == '__main__':
    if not ollama_direct_client.check_model_status():
        logger.warning(f"Modelo '{MODEL_NAME}' para acesso direto não encontrado. Instale com: ollama pull {MODEL_NAME}")
    
    print(f"Iniciando servidor Flask.")
    if agent_executor:
       
        logger.info(f"Agente customizado está ATIVO.")
    else:
        logger.warning(f"Agente customizado NÃO ESTÁ ATIVO. Endpoint /agent/invoke falhará. Verifique 'utils' e 'agentCreator'.")
    
    print(f"Servidor disponível em: http://localhost:5000")
    print(f"Health check: http://localhost:5000/health")
    app.run(host='0.0.0.0', port=5000, debug=True) 