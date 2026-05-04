"""
LLM service helpers.
"""

import importlib
from typing import Any, AsyncIterator, Dict, Iterator

from ..core.base import BaseService
from ..utils.exceptions import (
    AISDKException,
    APIException,
    ModelException,
    ProviderException,
    ValidationException,
)
from ..utils.helpers import format_response


class LLMService(BaseService):
    """Provider-backed LLM service with lazy provider loading."""

    def __init__(self, config):
        super().__init__(config)
        self._provider_specs: dict[str, tuple[str, str, str]] = {}
        self._register_providers()

    def _register_providers(self):
        self._provider_specs["alibaba"] = (
            "alibaba",
            "..providers.alibaba.llm_provider",
            "AlibabaLLMProvider",
        )
        self._provider_specs["deepseek"] = (
            "deepseek",
            "..providers.deepseek.llm_provider",
            "DeepSeekLLMProvider",
        )

    def _try_register_provider(
        self,
        provider_name: str,
        module_path: str,
        class_name: str,
    ) -> None:
        if provider_name in self.providers:
            return
        provider_config = self.config.get("providers", {}).get(provider_name, {})
        if not provider_config.get("enabled") or not provider_config.get("api_key"):
            return
        provider_class = self._load_provider_class(provider_name, module_path, class_name)
        self.register_provider(provider_name, provider_class)

    def _load_provider_class(
        self,
        provider_name: str,
        module_path: str,
        class_name: str,
    ):
        try:
            module = importlib.import_module(module_path, package=__package__)
            return getattr(module, class_name)
        except ModuleNotFoundError as exc:
            missing_name = str(getattr(exc, "name", "") or "").strip() or str(exc)
            raise ProviderException(provider_name, f"缺少依赖模块: {missing_name}")

    def _get_ready_provider(self, provider: str):
        if provider not in self.providers and provider in self._provider_specs:
            self._try_register_provider(*self._provider_specs[provider])
        if provider not in self.providers:
            available_providers = sorted(
                [
                    name
                    for name, cfg in self.config.get("providers", {}).items()
                    if cfg.get("enabled") and cfg.get("api_key")
                ]
            )
            raise ProviderException(
                provider,
                f"不支持的厂商，可用厂商: {', '.join(available_providers)}",
            )
        return self.get_provider(provider)

    def chat(self, provider: str, model: str, prompt: str, **kwargs) -> Dict[str, Any]:
        try:
            provider_instance = self._get_ready_provider(provider)
            validated_params = provider_instance.validate_params(kwargs)
            content = provider_instance.chat(model, prompt, **validated_params)
            return format_response(content, provider, model, is_stream=False)
        except (ProviderException, ModelException, AISDKException):
            raise
        except Exception as exc:
            raise APIException(f"聊天请求失败: {exc}")

    def chat_stream(
        self,
        provider: str,
        model: str,
        prompt: str,
        **kwargs,
    ) -> Iterator[Dict[str, Any]]:
        try:
            provider_instance = self._get_ready_provider(provider)
            validated_params = provider_instance.validate_params(kwargs)
            for content_chunk in provider_instance.chat_stream(
                model, prompt, **validated_params
            ):
                yield format_response(content_chunk, provider, model, is_stream=True)
        except (ProviderException, ModelException, AISDKException):
            raise
        except Exception as exc:
            raise APIException(f"流式聊天请求失败: {exc}")

    async def chat_async(
        self,
        provider: str,
        model: str,
        prompt: str,
        **kwargs,
    ) -> Dict[str, Any]:
        try:
            provider_instance = self._get_ready_provider(provider)
            validated_params = provider_instance.validate_params(kwargs)
            content = await provider_instance.chat_async(
                model, prompt, **validated_params
            )
            return format_response(content, provider, model, is_stream=False)
        except (ProviderException, ModelException, AISDKException):
            raise
        except Exception as exc:
            raise APIException(f"异步聊天请求失败: {exc}")

    async def chat_stream_async(
        self,
        provider: str,
        model: str,
        prompt: str,
        **kwargs,
    ) -> AsyncIterator[Dict[str, Any]]:
        try:
            provider_instance = self._get_ready_provider(provider)
            validated_params = provider_instance.validate_params(kwargs)
            async for content_chunk in provider_instance.chat_stream_async(
                model, prompt, **validated_params
            ):
                yield format_response(content_chunk, provider, model, is_stream=True)
        except (ProviderException, ModelException, AISDKException):
            raise
        except Exception as exc:
            raise APIException(f"异步流式聊天请求失败: {exc}")

    def get_available_providers(self) -> Dict[str, Dict[str, Any]]:
        available = {}
        for name, provider_config in self.config.get("providers", {}).items():
            if not provider_config.get("enabled"):
                continue
            available[name] = {
                "enabled": bool(provider_config.get("api_key")),
                "models": provider_config.get("models", {}),
                "description": provider_config.get("description", ""),
                "supports_stream": True,
            }
        return available

    def get_available_models(self, provider: str) -> list:
        provider_config = self.config.get("providers", {}).get(provider, {})
        return list(provider_config.get("models", {}).keys())

    def get_provider_info(self, provider: str) -> Dict[str, Any]:
        provider_config = self.config.get("providers", {}).get(provider, {})
        if not provider_config:
            raise ProviderException(provider, "不支持的厂商")
        return {
            "provider": provider,
            "available_models": self.get_available_models(provider),
            "default_params": provider_config.get("default_params", {}),
            "description": provider_config.get("description", ""),
        }

    def get_provider_models(self, provider: str) -> Dict[str, Any]:
        provider_config = self.config.get("providers", {}).get(provider, {})
        if not provider_config:
            raise ValidationException(f"未找到厂商 {provider}")
        return provider_config.get("models", {})
