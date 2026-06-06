"""Anthropic API client — single-shot JSON responses, no tool use."""

import json
import os
import time

import anthropic


class LLMClient:
    """
    Wraps Anthropic SDK for single-shot JSON responses.
    Runs synchronously inside ThreadPoolExecutor.
    Retries up to max_retry times with exponential backoff.
    """

    MODEL = 'claude-sonnet-4-6'
    MAX_TOKENS = 1024

    def __init__(self, max_retry: int = 3, retry_backoff_sec: float = 1.0):
        api_key = os.environ.get('ANTHROPIC_API_KEY')
        if not api_key:
            raise EnvironmentError('ANTHROPIC_API_KEY environment variable not set')
        self._client = anthropic.Anthropic(api_key=api_key)
        self._max_retry = max_retry
        self._retry_backoff = retry_backoff_sec

    def call(self, system_prompt: str, user_content: str) -> dict:
        """
        Call LLM and return parsed JSON dict.
        Raises RuntimeError if all retries fail or response is not valid JSON.
        """
        last_error = None
        for attempt in range(self._max_retry):
            try:
                response = self._client.messages.create(
                    model=self.MODEL,
                    max_tokens=self.MAX_TOKENS,
                    system=system_prompt,
                    messages=[{'role': 'user', 'content': user_content}],
                )
                text = response.content[0].text.strip()
                # JSON 블록 추출 (```json ... ``` 감싼 경우 대응)
                if text.startswith('```'):
                    text = text.split('```')[1]
                    if text.startswith('json'):
                        text = text[4:]
                return json.loads(text)
            except (json.JSONDecodeError, anthropic.APIError) as e:
                last_error = e
                if attempt < self._max_retry - 1:
                    time.sleep(self._retry_backoff * (2 ** attempt))
        raise RuntimeError(f'LLM call failed after {self._max_retry} retries: {last_error}')
