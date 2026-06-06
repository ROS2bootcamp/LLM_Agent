"""Gemini (Google Gen AI) client — single-shot JSON responses, no tool use.

API 키는 .env 파일의 GEMINI_API_KEY 를 참조한다(python-dotenv).
모델/토큰은 config(agent.yaml: llm.*)에서 주입된다.
"""

import json
import os
import time

from dotenv import load_dotenv
from google import genai
from google.genai import errors as genai_errors
from google.genai import types


class LLMClient:
    """
    Wraps the Google Gen AI SDK for single-shot JSON responses.
    Runs synchronously inside ThreadPoolExecutor.
    Retries up to max_retry times with exponential backoff.
    """

    DEFAULT_MODEL = 'gemini-2.5-flash'
    DEFAULT_MAX_TOKENS = 1024

    def __init__(
        self,
        model: str = DEFAULT_MODEL,
        max_tokens: int = DEFAULT_MAX_TOKENS,
        max_retry: int = 3,
        retry_backoff_sec: float = 1.0,
    ):
        # .env 에서 GEMINI_API_KEY 로드 (CWD 및 상위 디렉터리 탐색)
        load_dotenv()
        api_key = os.environ.get('GEMINI_API_KEY') or os.environ.get('GOOGLE_API_KEY')
        if not api_key:
            raise EnvironmentError(
                'GEMINI_API_KEY not set. .env 파일에 GEMINI_API_KEY=<your_key> 를 추가하세요.'
            )
        self._client = genai.Client(api_key=api_key)
        self._model = model
        self._max_tokens = max_tokens
        self._max_retry = max_retry
        self._retry_backoff = retry_backoff_sec

    def call(self, system_prompt: str, user_content: str) -> dict:
        """
        Call Gemini and return parsed JSON dict.
        Raises RuntimeError if all retries fail or response is not valid JSON.
        """
        last_error = None
        for attempt in range(self._max_retry):
            try:
                response = self._client.models.generate_content(
                    model=self._model,
                    contents=user_content,
                    config=types.GenerateContentConfig(
                        system_instruction=system_prompt,
                        max_output_tokens=self._max_tokens,
                        temperature=0.0,
                        response_mime_type='application/json',
                    ),
                )
                text = (response.text or '').strip()
                # JSON 블록 추출 (```json ... ``` 감싼 경우 대응)
                if text.startswith('```'):
                    text = text.split('```')[1]
                    if text.startswith('json'):
                        text = text[4:]
                return json.loads(text)
            except (json.JSONDecodeError, genai_errors.APIError) as e:
                last_error = e
                if attempt < self._max_retry - 1:
                    time.sleep(self._retry_backoff * (2 ** attempt))
        raise RuntimeError(f'LLM call failed after {self._max_retry} retries: {last_error}')
